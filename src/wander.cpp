#include <iostream>
#include <cmath>
#include <limits>

#include <ros/ros.h>
#include <angles/angles.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <tf/transform_datatypes.h>

#include "ca_msgs/Bumper.h"

// We can be in these states while wandering:
enum class State
{
  start,          // We're just starting out.
  look_accel,     // We're starting to look around, and we're accelerating.
  look,           // We're looking around.
  spin,           // We know where we're headed, and we're getting into position.
  spin_decel,     // We're almost in position.
  drive_accel,    // We're accelerating.
  drive,          // We're cruising.
  drive_decel,    // We're decelerating. Next=look_accel.
  emergency_stop  // We've stopped completely, and we're done.
};

// Helper for debugging. I'm sure there's some C++14 magic that I'm not grokking yet.
const char *toString(State state)
{
  switch (state)
  {
    case State::start: 
      return "start";
    case State::look_accel: 
      return "look_accel";
    case State::look: 
      return "look";
    case State::spin: 
      return "spin";
    case State::spin_decel: 
      return "spin_decel";
    case State::drive_accel: 
      return "drive_accel";
    case State::drive: 
      return "drive";
    case State::drive_decel: 
      return "drive_decel";
    case State::emergency_stop: 
      return "emergency_stop";
    // Omit default case to trigger compiler warning for missing cases.
  };
  return "missing";
}

// While we're looking we build a local map of directions. We have 1 bucket per degree.
class Bucket
{
private:

  double value;   // Cumulative value from the light sensor(s).
  double seconds; // How many seconds we were looking in this direction.

public:

  Bucket(): value {0.0}, seconds {0.0} {}
  void update(double v, double s) { value += v; seconds += s; }
  bool isEmpty() { return value == 0.0 && seconds == 0.0; }
  double normalized() { return seconds == 0.0 ? 0.0 : value / seconds; }
};

// The main class.
class Wander
{
private:

  static constexpr double linear_max_ {0.4};     // Cruising velocity
  static constexpr double linear_accel_ {0.1};   // Linear acceleration
  static constexpr double angular_max_ {0.2};    // Spinning velocity
  static constexpr double angular_accel_ {0.1};  // Angular acceleration

  // Time for a 360 spin, plus a bit more.
  static constexpr double look_time_ {2 * M_PI / angular_max_ + angular_max_ / angular_accel_};

  // How our light bumper behaves.
  static constexpr int light_bumper_center_ {355};
  static constexpr int light_bumper_range_ {10};

  // TODO can we use the timestamp in the header instead of a separate time?
  nav_msgs::Odometry last_odom_;
  ros::Time last_odom_time_;

  // TODO can we use the timestamp in the header instead of a separate time?
  ca_msgs::Bumper last_bumper_;
  ros::Time last_bumper_time_;

  // Last twist message we sent.
  geometry_msgs::Twist last_twist_;
  ros::Time last_twist_time_;

  State state_ {State::start};
  void transition(State state);

  // If we're looking:
  std::vector<Bucket> map_ {std::vector<Bucket>(360)};
  ros::Time start_looking_;               // When we started looking
  double start_yaw_;                      // Our yaw when we started looking
  double best_yaw_;                       // Best direction we found

  ros::Publisher cmd_vel_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber bumper_sub_;
  ros::Subscriber wheeldrop_sub_;

  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void bumperCallback(const ca_msgs::BumperConstPtr& msg);
  void wheeldropCallback(const std_msgs::EmptyConstPtr& msg);

public:

  Wander(ros::NodeHandle& nh);
  ~Wander() {}; // Suppress copy and move constructors
  void tick();
};

// Constructor.
Wander::Wander(ros::NodeHandle& nh)
{
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  odom_sub_ = nh.subscribe("odom", 1, &Wander::odomCallback, this);
  bumper_sub_ = nh.subscribe("bumper", 1, &Wander::bumperCallback, this);
  wheeldrop_sub_ = nh.subscribe("wheeldrop", 1, &Wander::wheeldropCallback, this);
}

// Helper: compute yaw from a quaternion message.
double getYaw(const geometry_msgs::Quaternion msg)
{
  tf::Quaternion q;
  tf::quaternionMsgToTF(msg, q);
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

// Transition to a new state.
void Wander::transition(State state)
{
  ROS_INFO("[WANDER] Transition from %s to %s", toString(state_), toString(state));
  state_ = state;

  // Initialize the new state.
  switch (state)
  {
    case State::look_accel:
      // Note the current yaw and time.
      start_yaw_ = getYaw(last_odom_.pose.pose.orientation);
      start_looking_ = last_odom_time_;

      // Reset our local map.
      std::fill(map_.begin(), map_.end(), Bucket());
      break;
      
    case State::emergency_stop:
      // Don't wait for the next tick -- publish a "dead stop" msg now.
      geometry_msgs::Twist msg;
      cmd_vel_pub_.publish(msg);
      break;
  }
}

// We're called at 10Hz. Publish a new velocity.
void Wander::tick()
{
  auto now = ros::Time::now();
  auto dt = (now - last_twist_time_).toSec();
  auto v0 = last_twist_.linear.x;
  auto a0 = last_twist_.angular.z;
  auto next_state = state_;
  geometry_msgs::Twist msg; // Init to 0, 0, 0, 0, 0, 0

  switch (state_)
  {
    case State::start:
      // Wait for messages to start arriving.
      if (last_odom_.header.seq > 0 && last_bumper_.header.seq > 0)
        next_state = State::look_accel;
      break;

    case State::look_accel:
      msg.angular.z = a0 + angular_accel_ * dt;
      if (msg.angular.z > angular_max_)
      {
        // At maximum angular velocity.
        msg.angular.z = angular_max_;
        next_state = State::look;
      }
      break;

    case State::look:
      if ((now - start_looking_).toSec() > look_time_)
      {
        // Time's up. Pick the best direction for forward motion and transition.
        int best_bucket {0};
        auto best_signal = std::numeric_limits<double>::max();

        for (int i = 0; i < map_.size(); ++i)
        {
          if (map_[i].isEmpty())
          {
            ROS_WARN("Empty bucket %d (we didn't go all the way around)", i);
            continue;
          }

          if (map_[i].normalized() < best_signal)
          {
            best_bucket = i;
            best_signal = map_[i].normalized();
          }
        }

        best_yaw_ = start_yaw_ + angles::from_degrees(best_bucket);

        // Start spinning into position. Maintain angular velocity for now.
        // TODO sense that we should decelerate immediately, and start doing so
        // TODO for 50% of the cases we should reverse the direction of spin and get there faster
        msg.angular.z = angular_max_;
        next_state = State::spin;
      }
      else
      {
        // Keep looking.
        msg.angular.z = angular_max_;
      }
      break;

    case State::spin:
    {
      // Did we spin into position?
      auto yaw = getYaw(last_odom_.pose.pose.orientation);
      if (std::abs(angles::shortest_angular_distance(start_yaw_, yaw)) < 0.2) // TODO
      {
        // Start slowing down immediately, and transition into decel mode.
        msg.angular.z = a0 - angular_accel_ * dt;
        next_state = State::spin_decel;
      }
      else
      {
        // Keep spinning.
        msg.angular.z = angular_max_;
      }
      break;
    }

    case State::spin_decel:
      msg.angular.z = a0 - angular_accel_ * dt;
      if (msg.angular.z < 0.0)
      {
        // We've stopped. Start driving.
        msg.angular.z = 0.0;
        next_state = State::drive_accel;
      }
      break;

    case State::drive_accel:
      msg.linear.x = v0 + linear_accel_ * dt;
      if (msg.linear.x > linear_max_)
      {
        // At maximum linear velocity.
        msg.linear.x = linear_max_;
        next_state = State::drive;
      }
      break;

    case State::drive:
      // Keep driving.
      msg.linear.x = linear_max_;
      break;

    case State::drive_decel:
      msg.linear.x = v0 - linear_accel_ * dt;
      if (msg.linear.x < 0.0)
      {
        // We've stopped. Start looking around.
        msg.linear.x = 0.0;
        next_state = State::look_accel;
      }
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
void Wander::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  last_odom_ = *msg;
  last_odom_time_ = ros::Time::now();
}

// Helper: did we hit something?
bool isCollision(const ca_msgs::BumperConstPtr& msg)
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
bool isTooClose(const ca_msgs::BumperConstPtr& msg)
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
void Wander::bumperCallback(const ca_msgs::BumperConstPtr& msg)
{
  auto now = ros::Time::now();
  auto dt = (now - last_bumper_time_).toSec();
  auto next_state = state_;

  // Check the bumper switches.
  if (state_ != State::emergency_stop && isCollision(msg))
  {
    next_state = State::emergency_stop;
  }
  else
  {
    // Process the IR signals -- the so-called "light bumper."
    switch (state_)
    {
      case State::look_accel:
      case State::look:
      {
        // Update our local map.
        auto yaw = getYaw(last_odom_.pose.pose.orientation);
        auto degrees = angles::to_degrees(angles::normalize_angle_positive(yaw - start_yaw_));
        int first_bucket = degrees - light_bumper_center_ - light_bumper_range_ / 2;
        int last_bucket = first_bucket + light_bumper_range_;
        for (int bucket = first_bucket; bucket < last_bucket; ++bucket)
        {
          int wrapped_bucket = bucket < map_.size() ? bucket : bucket - map_.size();
          map_[wrapped_bucket].update(msg->light_signal_front_left, dt);
        }
        break;
      }

      case State::drive_accel:
      case State::drive:
        // If we're too close, decelerate.
        if (isTooClose(msg))
          transition(State::drive_decel);
        break;

      default:
        break;
    }
  }

  // Save this message for future ref.
  last_bumper_ = *msg;
  last_bumper_time_ = now;

  // Execute any pending transitions.
  if (next_state != state_)
    transition(next_state);
}

// Handle wheeldrop messages.
void Wander::wheeldropCallback(const std_msgs::EmptyConstPtr& msg)
{
  ROS_WARN("[WANDER] Wheel drop");
  transition(State::emergency_stop);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wander");
  ros::NodeHandle nh;

  Wander wander{nh};

  ros::Rate r(10);
  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();
    wander.tick();
  }

  return 0;
}
