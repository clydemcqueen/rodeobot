#include <rodeobot/wander.h>
#include <angles/angles.h>
#include <tf/transform_datatypes.h>

// Helper: compute yaw from a quaternion message.
double getYaw(const geometry_msgs::Quaternion msg)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg, q);
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

// Helper: compute velocity.
double computeVelocity(bool accelerate, double v0, double a, double max_v)
{
  if (accelerate)
  {
    // Accelerate
    if (v0 + a > max_v)
      return max_v;
    else
      return v0 + a;
  }
  else
  {
    // Decelerate
    if (v0 - a < 0.0)
      return 0.0;
    else
      return v0 - a;
  }
}

// Initialize our scan map.
void Scan::init()
{
  std::fill(map_.begin(), map_.end(), 0.0);
}

// Put a value in our scan map.
void Scan::putValue(double yaw, double signal)
{
  auto degrees = static_cast<int>(angles::to_degrees(angles::normalize_angle_positive(yaw)));

  if (signal > map_[degrees])
    map_[degrees] = signal;
}

// Determine the best angle -- the angle with the lowest non-0 value.
double Scan::getBestAngle()
{
  int low_index{ 0 };
  auto low_value = std::numeric_limits<double>::max();

  for (int i = 0; i < map_.size(); ++i)
  {
    if (map_[i] > 0.0 && map_[i] < low_value)
    {
      low_index = i;
      low_value = map_[i];
    }
  }

  return angles::from_degrees(static_cast<double>(low_index));
}

const double DriveModel::max_v_{ 0.4 };
const double DriveModel::accel_{ 0.1 };

// Compute the desired linear velocity. Return 0 if we're at our goal.
double DriveModel::computeLinearX(double v0, double dt)
{
  return computeVelocity(driving_, v0, accel_ * dt, max_v_);
}

const double RotateModel::max_v_{ 0.2 };
const double RotateModel::accel_{ 0.1 };
const double RotateModel::epsilon_{ 2 * M_PI / 360 };

// Rotate to a particular angle.
void RotateModel::initGoalAngle(double current_yaw, double goal_yaw)
{
  goal_yaw_ = goal_yaw;
  full_circle_ = false;
  clockwise_ = angles::shortest_angular_distance(current_yaw, goal_yaw) > 0;
  suppress_goal_check_ = false;
}

// Rotate in place one full circle and end up in the same pose.
void RotateModel::initFullCircle(double current_yaw)
{
  goal_yaw_ = current_yaw;
  full_circle_ = true;
  clockwise_ = true;
  suppress_goal_check_ = true;
}


// Compute the angular velocity we want. Return 0 if we're at our goal.
double RotateModel::computeAngularZ(double current_yaw, double v0, double dt)
{
  auto error = std::abs(angles::shortest_angular_distance(current_yaw, goal_yaw_));

  // Have we reached our goal?
  if (!suppress_goal_check_ && error < epsilon_)
    return 0;

  // Can we stop suppressing the goal check?
  if (full_circle_ && suppress_goal_check_ && error > epsilon_)
    suppress_goal_check_ = false;

  // Compute the time it would take to stop at our current velocity.
  auto stopping_time = v0 / accel_;

  // Compute the stopping angle (think: stopping distance) at our current
  // velocity.
  auto stopping_yaw = v0 * stopping_time + 0.5 * accel_ * stopping_time * stopping_time;

  // Compute the desired angular velocity.
  auto a = accel_ * dt;
  if (clockwise_)
  {
    // Protect against wraparound.
    auto adj_goal = current_yaw > goal_yaw_ ? goal_yaw_ + 2 * M_PI : goal_yaw_;

    return computeVelocity(current_yaw < adj_goal - stopping_yaw, v0, a, max_v_);
  }
  else  // Counterclockwise
  {
    // Protect against wraparound.
    auto adj_curr = current_yaw < goal_yaw_ ? current_yaw + 2 * M_PI : current_yaw;

    return -computeVelocity(adj_curr > goal_yaw_ + stopping_yaw, -v0, a, max_v_);
  }
}

// Helper for debugging.
const char *toString(State state)
{
  switch (state)
  {
    case State::start:
      return "start";
    case State::look:
      return "look";
    case State::spin:
      return "spin";
    case State::drive:
      return "drive";
    case State::emergency_stop:
      return "emergency_stop";
      // Omit default case to trigger compiler warning for missing cases.
  };
  return "missing";
}

// Constructor.
Wander::Wander(ros::NodeHandle &nh)
{
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  odom_sub_ = nh.subscribe("odom", 1, &Wander::odomCallback, this);
  bumper_sub_ = nh.subscribe("bumper", 1, &Wander::bumperCallback, this);
  wheeldrop_sub_ = nh.subscribe("wheeldrop", 1, &Wander::wheeldropCallback, this);
}

// Transition to a new state.
void Wander::transition(State state)
{
  ROS_INFO("[WANDER] Transition from %s to %s", toString(state_), toString(state));
  state_ = state;

  // Initialize the new state.
  switch (state)
  {
    case State::look:
      start_yaw_ = last_yaw_;
      scan_.init();
      rotate_model_.initFullCircle(last_yaw_);
      break;

    case State::spin:
      rotate_model_.initGoalAngle(last_yaw_, start_yaw_ + scan_.getBestAngle());
      break;

    case State::drive:
      drive_model_.initDrive();
      break;

    case State::emergency_stop:
      // Don't wait for the next spinOnce -- publish a "dead stop" msg now.
      geometry_msgs::Twist msg;
      cmd_vel_pub_.publish(msg);
      break;
  }
}

// We're called at 10Hz. Publish a new velocity.
void Wander::spinOnce()
{
  auto now = ros::Time::now();
  auto dt = (now - last_twist_time_).toSec();
  auto next_state = state_;
  geometry_msgs::Twist msg;  // Init to 0, 0, 0, 0, 0, 0

  switch (state_)
  {
    case State::start:
      if (last_odom_.header.seq > 0 && last_bumper_.header.seq > 0)
        next_state = State::look;
      break;

    case State::look:
      msg.angular.z = rotate_model_.computeAngularZ(last_yaw_, last_twist_.angular.z, dt);
      if (msg.angular.z == 0.0)
        next_state = State::spin;
      break;

    case State::spin:
      msg.angular.z = rotate_model_.computeAngularZ(last_yaw_, last_twist_.angular.z, dt);
      if (msg.angular.z == 0.0)
        next_state = State::drive;
      break;

    case State::drive:
      msg.linear.x = drive_model_.computeLinearX(last_twist_.linear.x, dt);
      if (msg.linear.x == 0.0)
        next_state = State::look;
      break;

    default:
      break;
  }

  // Publish a cmd_vel message.
  cmd_vel_pub_.publish(msg);
  last_twist_ = msg;
  last_twist_time_ = now;

  // Execute any pending transitions.
  if (next_state != state_)
    transition(next_state);
}

// Handle odometry messages.
void Wander::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  last_yaw_ = getYaw(msg->pose.pose.orientation);
}

// Helper: did we hit something?
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

// Helper: are we really close to hitting something?
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

// Handle bumper messages.
void Wander::bumperCallback(const ca_msgs::BumperConstPtr &msg)
{
  // Check the bumper switches.
  if (state_ != State::emergency_stop && isCollision(msg))
  {
    transition(State::emergency_stop);
    return;
  }

  // Process the IR signals -- the so-called "light bumper."
  switch (state_)
  {
    case State::look:
      scan_.putValue(last_yaw_ - start_yaw_, msg->light_signal_front_left);
      break;

    case State::drive:
      if (isTooClose(msg))
        drive_model_.initStop(); // Stop gracefully
      break;

    default:
      break;
  }

  // Save this message for future ref.
  last_bumper_ = *msg;
  last_bumper_time_ = ros::Time::now();
}

// Handle wheeldrop messages.
void Wander::wheeldropCallback(const std_msgs::EmptyConstPtr &msg)
{
  ROS_WARN("[WANDER] Wheel drop");
  transition(State::emergency_stop);
}
