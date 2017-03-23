#include <rodeobot/w3.h>
#include <angles/angles.h>

namespace rodeobot {

// TODO add turtlebot bumpers

//===========================================================================
// Utilities
//===========================================================================

// Return a smooth velocity. Works for +/foward/CCW and -/backward/CW.
double smooth(double goal, double v, double accel, double min, double max)
{
  assert(min > 0.0);
  assert(max > 0.0);
  assert(max > min);

  accel /= SPIN_RATE;

  if (std::abs(v - goal) < accel)
  {
    // Close enough -- pin to goal
    v = goal;
  }
  else
  {
    // Accelerate / decelerate
    if (v < goal)
      v += accel;
    else if (v > goal)
      v -= accel;
  }

  // Clip to max
  if (std::abs(v) > max)
    v = (v > 0.0) ? max : -max;

  // Clip to min
  if (std::abs(v) < min)
    v = 0.0;

  // ROS_INFO("goal %f, accel %f, min %f, max %f, v %f", goal, accel, min, max, v);
  return v;
}

// Return true if we hit something.
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

// Return true if we almost hit something.
bool isTooClose(const ca_msgs::BumperConstPtr &msg)
{
  if (msg->is_light_left)
  {
    ROS_WARN("[WANDER] Too close: left light bumper");
    return true;
  }

  if (msg->is_light_front_left)
  {
    ROS_WARN("[WANDER] Too close: front left light bumper");
    return true;
  }

  if (msg->is_light_center_left)
  {
    ROS_WARN("[WANDER] Too close: center left light bumper");
    return true;
  }

  if (msg->is_light_right)
  {
    ROS_WARN("[WANDER] Too close: right light bumper");
    return true;
  }

  if (msg->is_light_front_right)
  {
    ROS_WARN("[WANDER] Too close: front right light bumper");
    return true;
  }

  if (msg->is_light_center_right)
  {
    ROS_WARN("[WANDER] Too close: center right light bumper");
    return true;
  }

  return false;
}

//===========================================================================
// Laser scan utilities
//===========================================================================

// Find the widest max/inf/nan arc.
void find(sensor_msgs::LaserScan &scan, float max, int &start, int &width)
{
  width = 0;

  // -1 means "not an arc"
  start = -1;
  int current_start = -1;

  for (int i = 0; i < scan.ranges.size() + 1; ++i) // Note: size()+1 iterations
  {
    if ((i < scan.ranges.size()) && (std::isinf(scan.ranges[i]) || std::isnan(scan.ranges[i]) || scan.ranges[i] >= max))
    {
      // In an arc
      if (current_start < 0)
      {
        current_start = i; // Start a new arc
      }
    }
    else 
    {
      // Not in an arc
      if (current_start >= 0)
      {
        // Examine the arc that just ended
        if (i - current_start > width)
        {
          start = current_start;
          width = i - current_start;
        }
        current_start = -1;
      }
    }
  }

  // ROS_INFO("size %lu start %d width %d min %f max %f inc %f", scan.ranges.size(), start, width, 
  //   scan.angle_min, scan.angle_max, scan.angle_increment);
}

// Draw an arc so we can see it in rviz.
void draw(sensor_msgs::LaserScan &scan, float max, int start, int width)
{
  for (int i = 0; i < scan.ranges.size(); ++i)
    scan.ranges[i] = (i >= start && i < start + width) ?  max : NAN;
}

//===========================================================================
// Wander
//===========================================================================

Wander::Wander(ros::NodeHandle &nh, tf::TransformListener &tf):
  nh_{nh},
  tf_{tf},
  state_{State::start},
  good_scan_{false}
{
  nh.getParam("x_min_v", x_min_v_);
  nh.getParam("x_max_v", x_max_v_);
  nh.getParam("x_accel", x_accel_);
  nh.getParam("r_min_v", r_min_v_);
  nh.getParam("r_max_v", r_max_v_);
  nh.getParam("r_accel", r_accel_);
  nh.getParam("r_epsilon", r_epsilon_);
  nh.getParam("scan_horizon", scan_horizon_);
  nh.getParam("scan_width", scan_width_);

  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  laser_pub_ = nh.advertise<sensor_msgs::LaserScan>("scan1", 1);
  laser_sub_ = nh.subscribe("/scan", 1, &Wander::laserCallback, this);
  bumper_sub_ = nh.subscribe("/bumper", 1, &Wander::bumperCallback, this);
  wheeldrop_sub_ = nh.subscribe("/wheeldrop", 1, &Wander::wheeldropCallback, this);
}

void Wander::emergencyStop()
{
  geometry_msgs::Twist msg;
  cmd_vel_pub_.publish(msg);
  state_ = State::emergency_stop;
}

void Wander::laserCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
  // Don't block -- just skip this reading
  if (scan_mutex_.try_lock())
  {
    scan_ = *msg;
    scan_mutex_.unlock();
    good_scan_ = true;
  }
}

void Wander::bumperCallback(const ca_msgs::BumperConstPtr &msg)
{
  if (state_ != State::emergency_stop && (isCollision(msg) || isTooClose(msg)))
    emergencyStop();
}

void Wander::wheeldropCallback(const std_msgs::EmptyConstPtr &msg)
{
  ROS_WARN("[WANDER] Wheel drop");
  emergencyStop();
}

void Wander::spinOnce(const ros::TimerEvent &event)
{
  if ((event.current_real - event.current_expected).toSec() > 2.0 / SPIN_RATE)
    ROS_WARN("This event is %f clock seconds late", (event.current_real - event.current_expected).toSec());

  if (event.profile.last_duration.toSec() > 1.0 / SPIN_RATE)
    ROS_WARN("Last event took %f wall seconds too long", event.profile.last_duration.toSec() - 1.0 / SPIN_RATE);

  if (state_ == State::emergency_stop)
    return;

  if (state_ == State::start && good_scan_)
    state_ = State::drive;

  if (state_ == State::start)
    return;

  // Find the widest arc.
  scan_mutex_.lock();
  int arc_start, arc_width;
  find(scan_, scan_horizon_, arc_start, arc_width);
  if (arc_width > 0)
  {
    draw(scan_, scan_horizon_, arc_start, arc_width);
    laser_pub_.publish(scan_);
  }
  scan_mutex_.unlock();

  double x_target_v = 0.0;
  double r_target_v = 0.0;

  // Calculate our motion.
  if (state_ == State::drive)
    driveMotion(arc_start, arc_width, x_target_v, r_target_v);
  else
    recoverMotion(arc_start, arc_width, x_target_v, r_target_v);

  // Smooth the motion.
  x_prev_v_ = smooth(x_target_v, x_prev_v_, x_accel_, x_min_v_, x_max_v_);
  r_prev_v_ = smooth(r_target_v, r_prev_v_, r_accel_, r_min_v_, r_max_v_);
  
  // Publish a velocity command.
  geometry_msgs::Twist twist;
  twist.linear.x = x_prev_v_;
  twist.angular.z = r_prev_v_;
  cmd_vel_pub_.publish(twist);
}

void Wander::driveMotion(int arc_start, int arc_width, double &x_target_v, double &r_target_v)
{
  double arc_d = std::sin(arc_width / 2 * scan_.angle_increment) * scan_horizon_;

  if (arc_d < scan_width_)
  {
    ROS_WARN("Too small! %f", arc_d);
    state_ = State::recover;
  }
  else
  {
    // Calculate goal yaw.
    double goal_yaw = scan_.angle_min + scan_.angle_increment * (arc_start + arc_width / 2);
    // ROS_INFO("Goal yaw %f", goal_yaw);

    // Calculate target velocity.
    x_target_v = x_max_v_;
    r_target_v = 0.0;
    if (goal_yaw > r_epsilon_)
      r_target_v = r_max_v_;
    else if (goal_yaw < -r_epsilon_)
      r_target_v = -r_max_v_;
  }
}

void Wander::recoverMotion(int arc_start, int arc_width, double &x_target_v, double &r_target_v)
{
  if (arc_width == scan_.ranges.size())
  {
    ROS_WARN("Drive!");
    dir_ = Direction::none;
    state_ = State::drive;
  }
  else
  {
    // Pick a direction.
    if (dir_ == Direction::none)
    {
      dir_ = (rand() % 2 > 0) ? Direction::cw : Direction::ccw;
      ROS_INFO("Turning %s", dir_ == Direction::cw ? "clockwise" : "counterclockwise");
    }

    // Calculate angular velocity. Leave linear velocity at 0.0.
    r_target_v = (dir_ == Direction::ccw) ? r_max_v_ : -r_max_v_;
  }
}

} // namespace rodeobot
