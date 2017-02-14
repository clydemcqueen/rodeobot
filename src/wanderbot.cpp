#include <rodeobot/wanderbot.h>
#include <chrono>
#include <random>
#include <tf/transform_datatypes.h>

namespace rodeobot {

// Constructor.
Wanderbot::Wanderbot(ros::NodeHandle &nh, tf::TransformListener &tf):
  nh_{nh},
  tf_{tf},
  costmap_ros_{nullptr},
  driver_{nullptr},
  state_{State::start}
{
  ROS_INFO("Wanderbot initializing");

  // Create our costmap.
  costmap_ros_ = new costmap_2d::Costmap2DROS("wander_costmap", tf_); // TODO param name

  // Shift the costmap so that (0,0) is in the center.
  // TODO grab the mutex?
  auto costmap = costmap_ros_->getCostmap();
  costmap->updateOrigin(costmap->getSizeInMetersX() / -2.0, costmap->getSizeInMetersY() / -2.0);

  // Create our local planner. In ROS terminology, our planners is the global planner.
  driver_ = new dwa_local_planner::DWAPlannerROS();
  driver_->initialize("wander_planner", &tf_, costmap_ros_); // TODO param name

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  bumper_sub_ = nh_.subscribe("bumper", 1, &Wanderbot::bumperCallback, this);
  wheeldrop_sub_ = nh_.subscribe("wheeldrop", 1, &Wanderbot::wheeldropCallback, this);

  ROS_INFO("Wanderbot running");
}

// Destructor.
Wanderbot::~Wanderbot()
{
  if (driver_ != nullptr)
    delete driver_;

  if (costmap_ros_ != nullptr)
    delete costmap_ros_;
}

// Big trouble, stop!
void Wanderbot::emergencyStop()
{
  ROS_ERROR("Emergency stop");
  state_ = State::emergency_stop;

  // Don't wait for the next spinOnce -- publish a "dead stop" msg now.
  geometry_msgs::Twist msg;
  cmd_vel_pub_.publish(msg);
}

// Check for free space.
bool free(costmap_2d::Costmap2D *costmap, int cell_x, int cell_y)
{
  // TODO It's possible for the costmap to change while we're reading; should lock the mutex.
 
  // How much open space we want. TODO param
  static const int radius = 15;

  int size_x = costmap->getSizeInCellsX(); // Unsigned int to int
  int size_y = costmap->getSizeInCellsY();

  for (int x = cell_x - radius; x < cell_x + radius; ++x)
  {
    for (int y = cell_y - radius; y < cell_y + radius; ++y)
    {
      if (x >= 0 && y >= 0 && x < size_x && y < size_y)
      {
        auto cost = costmap->getCost(x, y);
        if (cost != costmap_2d::NO_INFORMATION && cost != costmap_2d::FREE_SPACE)
        {
          return false;
        }
      }
    }
  }

  return true;
}

// Generate a simple global plan.
bool plan(costmap_2d::Costmap2DROS *costmap_ros, std::vector<geometry_msgs::PoseStamped> &plan)
{
  tf::Stamped<tf::Pose> current_pose;
  if (!costmap_ros->getRobotPose(current_pose))
  {
    ROS_ERROR("Can't get robot pose");
    return false;
  }

  // Assert that we're in the odom frame. The alternative is to transform.
  ROS_ASSERT(current_pose.frame_id_ == "odom");

  // Clear the plan.
  plan.clear();

  // Point 0 of our plan is our current pose.
  geometry_msgs::PoseStamped pose_msg;
  tf::poseStampedTFToMsg(current_pose, pose_msg);
  plan.push_back(pose_msg);

  // How many times will we try to find a goal?
  static const int max_tries = 20;

  // Create a random number generator.
  static unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  static std::default_random_engine generator(seed);

  costmap_2d::Costmap2D *costmap = costmap_ros->getCostmap();

  // ROS_DEBUG("Costmap size is %d X %d cells, %f X %f meters", costmap->getSizeInCellsX(), costmap->getSizeInCellsY(), 
  //    costmap->getSizeInMetersX(), costmap->getSizeInMetersY());

  // Create two distributions.
  std::uniform_int_distribution<int> gen_x(0, costmap->getSizeInCellsX() - 1);
  std::uniform_int_distribution<int> gen_y(0, costmap->getSizeInCellsY() - 1);

  for (int t = 0; t < max_tries; ++t)
  {
    // Pick a random cell in the map.
    int cell_x = gen_x(generator);
    int cell_y = gen_y(generator);

    // Transform from cell to world coordinates.
    double world_x, world_y;
    costmap->mapToWorld(cell_x, cell_y, world_x, world_y);

    double dx = world_x - pose_msg.pose.position.x;
    double dy = world_y - pose_msg.pose.position.y;
    double h = hypot(dx, dy);

    // Too close?
    if (h < 1.0)
      continue;

    // Too crowded?
    if (!free(costmap, cell_x, cell_y))
      continue;

    // Add the goal pose to the plan. Frame is still "odom". Timestamp doesn't matter.
    pose_msg.pose.position.x = world_x;
    pose_msg.pose.position.y = world_y;
    pose_msg.pose.position.z = 0;
    pose_msg.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, acos(dx / h));
    plan.push_back(pose_msg);

    return true;
  }

  // Couldn't generate a plan.
  return false;
}

// Rotate in place.
void rotate(geometry_msgs::Twist& msg)
{
  msg.angular.z = 0.4; // TODO param
}

// Heartbeat; called 10 times per second.
void Wanderbot::spinOnce(const ros::TimerEvent &event)
{
  // ROS_DEBUG("Current expected %f clock seconds, current real %f clock seconds", event.current_expected.toSec(), event.current_real.toSec());
  if (event.current_real - event.current_expected > ros::Duration(0.2))
    ROS_WARN("This event called %f clock seconds late -- expect jerky motion", (event.current_real - event.current_expected).toSec());

  // ROS_DEBUG("Last event took %f wall seconds", event.profile.last_duration.toSec());
  if (event.profile.last_duration > ros::WallDuration(0.1))
    ROS_WARN("Last event took too long! %f wall seconds", event.profile.last_duration.toSec());

  auto next_state = state_;
  geometry_msgs::Twist msg;  // Init to 0, 0, 0, 0, 0, 0

  switch (state_)
  {
    case State::start:
      state_ = State::plan;
      break;

    case State::plan:
      if (plan(costmap_ros_, plan_))
      {
        ROS_INFO("New goal found => drive to %f, %f", plan_[1].pose.position.x, plan_[1].pose.position.y);
        driver_->setPlan(plan_);
        state_ = State::drive;
        last_plan_time_ = ros::Time::now();
      }
      else
      {
        ROS_WARN("Failed to find a new goal => plan again");
      }
      break;

    case State::drive:
      if (driver_->isGoalReached())
      {
        ROS_INFO("Goal reached => plan");
        state_ = State::plan;
      }
      else if (last_plan_time_ + ros::Duration(60) < ros::Time::now())
      {
        ROS_WARN("Timeout => plan");
        state_ = State::plan;
      }
      else if (!driver_->computeVelocityCommands(msg))
      {
        ROS_WARN("Can't move => recover, then plan");
        state_ = State::recover;
      }
      break;

    case State::recover:
      // Super simple recovery: rotate in place just a bit clockwise, then try a new plan.
      // Eventually we'll rotate all the way around. If that doesn't work, we're really stuck.
      rotate(msg);
      state_ = State::plan;
      break;

    default:
      break;
  }

  // Publish the cmd_vel message.
  cmd_vel_pub_.publish(msg);
}

// Did we hit something?
bool isCollision(const ca_msgs::BumperConstPtr &msg)
{
  if (msg->is_left_pressed)
  {
    ROS_ERROR("Hit left bumper");
    return true;
  }

  if (msg->is_right_pressed)
  {
    ROS_ERROR("Hit right bumper");
    return true;
  }

  return false;
}

// Are we really close to hitting something?
bool isTooClose(const ca_msgs::BumperConstPtr &msg)
{
  if (msg->is_light_left)
  {
    ROS_ERROR("Too close: left light bumper");
    return true;
  }

  if (msg->is_light_front_left)
  {
    ROS_ERROR("Too close: front left light bumper");
    return true;
  }

  if (msg->is_light_center_left)
  {
    ROS_ERROR("Too close: center left light bumper");
    return true;
  }

  if (msg->is_light_right)
  {
    ROS_ERROR("Too close: right light bumper");
    return true;
  }

  if (msg->is_light_front_right)
  {
    ROS_ERROR("Too close: front right light bumper");
    return true;
  }

  if (msg->is_light_center_right)
  {
    ROS_ERROR("Too close: center right light bumper");
    return true;
  }

  return false;
}

// Handle bumper message.
void Wanderbot::bumperCallback(const ca_msgs::BumperConstPtr &msg)
{
  if (state_ != State::emergency_stop && (isCollision(msg) || isTooClose(msg)))
    emergencyStop();
}

// Handle wheeldrop message.
void Wanderbot::wheeldropCallback(const std_msgs::EmptyConstPtr &msg)
{
  if (state_ != State::emergency_stop)
  {
    ROS_ERROR("Wheel drop");
    emergencyStop();
  }
}

} // namespace rodeobot
