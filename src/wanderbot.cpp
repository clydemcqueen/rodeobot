#include <rodeobot/wanderbot.h>
#include <random>
#if 0
#include <angles/angles.h> // TODO?
#include <tf/transform_datatypes.h> // TODO?
#endif

namespace rodeobot {

#if 0
// Compute yaw from a quaternion message.
double getYaw(const geometry_msgs::Quaternion msg) // TODO?
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg, q);
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}
#endif

// Enum to string.
const char *toString(State state)
{
  switch (state)
  {
    case State::start:
      return "start";
    case State::planning:
      return "planning";
    case State::driving:
      return "driving";
    case State::emergency_stop:
      return "emergency_stop";
      // Omit default case to trigger compiler warning for missing cases.
  };
  return "missing";
}

// Constructor.
Wanderbot::Wanderbot(ros::NodeHandle &nh, tf::TransformListener &tf):
  nh_{nh},
  tf_{tf},
  costmap_{nullptr},
  driver_{nullptr}
{
  // TODO get params needed for costmap & dwa planner
  // TODO allow for reconfiguration
  
#if 0
  nh.getParam("drive_min_v", drive_model_.min_v_);
  nh.getParam("drive_max_v", drive_model_.max_v_);
  nh.getParam("drive_accel", drive_model_.accel_);

  ROS_INFO("[WANDER] Drive min %f max %f accel %f", 
    drive_model_.min_v_, drive_model_.max_v_, drive_model_.accel_);

  nh.getParam("rotate_min_v", rotate_model_.min_v_);
  nh.getParam("rotate_max_v", rotate_model_.max_v_);
  nh.getParam("rotate_accel", rotate_model_.accel_);
  nh.getParam("rotate_epsilon", rotate_model_.epsilon_);

  ROS_INFO("[WANDER] Rotate min %f max %f accel %f epsilon %f", 
    rotate_model_.min_v_, rotate_model_.max_v_, rotate_model_.accel_, rotate_model_.epsilon_);
#endif

  costmap_ = new costmap_2d::Costmap2DROS("wander_costmap", tf_);
  driver_ = new dwa_local_planner::DWAPlannerROS();
  driver_->initialize("wander_planner", &tf_, costmap_);

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
#if 0
  odom_sub_ = nh_.subscribe("odom", 1, &Wanderbot::odomCallback, this); // TODO?
  bumper_sub_ = nh_.subscribe("bumper", 1, &Wanderbot::bumperCallback, this);
  wheeldrop_sub_ = nh_.subscribe("wheeldrop", 1, &Wanderbot::wheeldropCallback, this);
#endif

  ROS_INFO("[WANDER] Wanderbot awake!");
}

// Destructor.
Wanderbot::~Wanderbot()
{
  if (driver_ != nullptr)
    delete driver_;

  if (costmap_ != nullptr)
    delete costmap_;
}

// Transition to a new state.
void Wanderbot::transition(State state)
{
  ROS_INFO("[WANDER] Transition from %s to %s", toString(state_), toString(state));
  state_ = state;

  // Initialize the new state.
  switch (state)
  {
    case State::emergency_stop:
      // Don't wait for the next spinOnce -- publish a "dead stop" msg now.
      geometry_msgs::Twist msg;
      cmd_vel_pub_.publish(msg);
      break;
  }
}

// Check for free space.
bool free(costmap_2d::Costmap2D *map, int goal_x, int goal_y)
{
  static const int radius = 17; // TODO convert from cm to cells, also make a param

  int size_x = map->getSizeInCellsX(); // Unsigned int to int
  int size_y = map->getSizeInCellsY();

  for (int x = goal_x - radius; x < goal_x + radius; ++x)
  {
    for (int y = goal_y - radius; y < goal_y + radius; ++y)
    {
      if (x >= 0 && y >= 0 && x < size_x && y < size_y)
      {
        auto cost = map->getCost(x, y);
        if (cost != costmap_2d::NO_INFORMATION && cost != costmap_2d::FREE_SPACE)
        {
          return false;
        }
      }
    }
  }

  return true;
}

// Generate a very simple plan w/ 1 pose -- our goal pose.
bool plan(costmap_2d::Costmap2DROS *costmap, std::vector<geometry_msgs::PoseStamped> &plan)
{
  // Clear any previous poses.
  plan.clear();

  static const int max_tries = 20;
  static std::random_device rd;

  costmap_2d::Costmap2D *map = costmap->getCostmap();
  std::uniform_int_distribution<int> gen_x(0, map->getSizeInCellsX() - 1);
  std::uniform_int_distribution<int> gen_y(0, map->getSizeInCellsY() - 1);

  ROS_INFO("[WANDER] planning; costmap size is %d X %d", map->getSizeInCellsX(), map->getSizeInCellsY());

  for (int t = 0; t < max_tries; ++t)
  {
    // Pick a random point on the map.
    int goal_x = gen_x(rd);
    int goal_y = gen_y(rd);

    // Is it far enough away to be interesting?
    // TODO need current pose to do this calc

    // Is there enough free or unknown space around it?
    if (free(map, goal_x, goal_y))
    {
      // This is our new goal. Add it to the plan.
      geometry_msgs::PoseStamped goal;
      goal.pose.position.x = goal_x;
      goal.pose.position.y = goal_y;
      goal.pose.position.z = 0;
      // TODO set orientation as well
      goal.header.frame_id = "odom"; // TODO hack, should take from argument
      plan.push_back(goal);
      return true;
    }
  }

  // Couldn't generate a plan.
  return false;
}

// Heartbeat; called 10 times per second.
void Wanderbot::spinOnce()
{
#if 0
  auto now = ros::Time::now();
  auto dt = (now - last_twist_time_).toSec();
  ROS_INFO("[WANDER] dt %f", dt);
#endif
  auto next_state = state_;
  geometry_msgs::Twist msg;  // Init to 0, 0, 0, 0, 0, 0

  switch (state_)
  {
    case State::start:
#if 0
      if (saw_odom_ && saw_bumper_)
#endif
        next_state = State::planning;
      break;

    case State::planning:
      if (plan(costmap_, plan_))
      {
        ROS_INFO("[WANDER] New goal found: %f, %f", plan_[0].pose.position.x, plan_[0].pose.position.y);
        driver_->setPlan(plan_);
        next_state = State::driving;
      }
      else
      {
        ROS_INFO("[WANDER] Failed to find a new goal, will try again");
        // TODO try recovery strategies
      }
      break;

    case State::driving:
      if (driver_->isGoalReached())
      {
        ROS_INFO("[WANDER] Goal reached");
        next_state = State::planning;
      }
      else if (!driver_->computeVelocityCommands(msg))
      {
        ROS_INFO("[WANDER] Can't move");
        // TODO try recovery strategies
        next_state = State::planning;
      }
      break;

    default:
      break;
  }

  // Publish the cmd_vel message.
  cmd_vel_pub_.publish(msg);
#if 0
  last_twist_ = msg;
  last_twist_time_ = now;
#endif

  // Execute any pending transitions.
  if (next_state != state_)
    transition(next_state);

  // TODO did we take longer than 0.1s? if so, bark
}

#if 0
// Handle odom message.
void Wanderbot::odomCallback(const nav_msgs::OdometryConstPtr &msg) // TODO?
{
  saw_odom_ = true;
  last_yaw_ = getYaw(msg->pose.pose.orientation); // TODO?
}
#endif

// Did we hit something?
bool isCollision(const ca_msgs::BumperConstPtr &msg)
{
  if (msg->is_left_pressed)
  {
    ROS_WARN("[WANDER] Hit left bumper");
    return true;
  }

  if (msg->is_right_pressed)
  {
    ROS_WARN("[WANDER] Hit right bumper");
    return true;
  }

  return false;
}

// Are we really close to hitting something?
bool isTooClose(const ca_msgs::BumperConstPtr &msg)
{
  if (msg->is_light_left)
  {
    ROS_INFO("[WANDER] Too close: left light bumper");
    return true;
  }

  if (msg->is_light_front_left)
  {
    ROS_INFO("[WANDER] Too close: front left light bumper");
    return true;
  }

  if (msg->is_light_center_left)
  {
    ROS_INFO("[WANDER] Too close: center left light bumper");
    return true;
  }

  if (msg->is_light_right)
  {
    ROS_INFO("[WANDER] Too close: right light bumper");
    return true;
  }

  if (msg->is_light_front_right)
  {
    ROS_INFO("[WANDER] Too close: front right light bumper");
    return true;
  }

  if (msg->is_light_center_right)
  {
    ROS_INFO("[WANDER] Too close: center right light bumper");
    return true;
  }

  return false;
}

// Handle bumper message.
void Wanderbot::bumperCallback(const ca_msgs::BumperConstPtr &msg)
{
  saw_bumper_ = true;

  // Check the bumper switches.
  if (state_ != State::emergency_stop && isCollision(msg))
  {
    transition(State::emergency_stop);
    return;
  }

  // Process the IR signals -- the so-called "light bumper."
  switch (state_)
  {
#if 0
    case State::look:
      scan_.putValue(last_yaw_ - start_yaw_, msg->light_signal_front_left);
      break;

    case State::drive:
      if (isTooClose(msg))
        drive_model_.initStop(); // Stop gracefully
      break;
#endif

    default:
      break;
  }
}

// Handle wheeldrop message.
void Wanderbot::wheeldropCallback(const std_msgs::EmptyConstPtr &msg)
{
  ROS_WARN("[WANDER] Wheel drop");
  transition(State::emergency_stop);
}

} // namespace rodeobot
