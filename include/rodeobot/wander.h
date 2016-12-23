#ifndef RODEOBOT_WANDER_H
#define RODEOBOT_WANDER_H 

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include "ca_msgs/Bumper.h"

namespace rodeobot {

//===========================================================================
// Radial map of our environment.
//===========================================================================

class Scan
{
private:
  std::vector<double> map_{ std::vector<double>(360) };

public:
  // Initialize our scan map.
  void init();

  // Put a value in our scan map.
  void putValue(double yaw, double signal);

  // Determine the best angle -- the angle with the lowest non-0 value.
  // TODO convert to distance in meters so we can combine laser data
  double getBestAngle();
};

//===========================================================================
// Motion model: drive straight ahead.
//===========================================================================

class DriveModel
{
private:
  bool driving_{ false };

public:
  // ROS parameters. TODO do these need to be public?
  double min_v_{ 0.1 };
  double max_v_{ 0.3 };
  double accel_{ 0.3 };

  bool driving() { return driving_; }

  void initDrive() { driving_ = true; }
  void initStop() { driving_ = false; }

  // Compute the desired linear velocity. Return 0 if we're at our goal.
  double computeLinearX(double v0, double dt);
};

//===========================================================================
// Motion model: rotate in place.
//===========================================================================

class RotateModel
{
private:
  double goal_yaw_{ 0.0 };            // Goal yaw
  bool full_circle_{ false };         // True: rotate one full circle
  bool clockwise_{ false };           // True: go clockwise
  bool suppress_goal_check_{ false }; // True: suppress the goal check for now

public:
  // ROS parameters. TODO do these need to be public?
  double min_v_{ 0.1 };
  double max_v_{ 0.4 };
  double accel_{ 0.4 };
  double epsilon_{ 0.02 };

  bool fullCircle() { return full_circle_; }
  bool clockwise() { return clockwise_; }

  // Rotate to a particular angle.
  void initGoalAngle(double current_yaw, double goal_yaw);

  // Rotate in place one full circle and end up in the same pose.
  void initFullCircle(double current_yaw);

  // Compute the angular velocity we want. Return 0 if we're at our goal.
  double computeAngularZ(double yaw, double v0, double dt);
};

//===========================================================================
// Wander about.
//===========================================================================

enum class State
{
  start,          // We're just starting out.
  look,           // We're looking around.
  spin,           // We know where we're headed, and we're getting into position.
  drive,          // We're cruising.
  emergency_stop  // We've stopped completely, and we're done.
};

class Wander
{
private:
  // Used for deciding when to start.
  bool saw_odom_{ false };
  bool saw_bumper_{ false };

  // Last yaw we saw.
  double last_yaw_{ 0.0 };

  // Last twist we sent.
  geometry_msgs::Twist last_twist_;
  ros::Time last_twist_time_;

  // Internal state.
  State state_{ State::start };
  double start_yaw_{ 0.0 };
  RotateModel rotate_model_;
  DriveModel drive_model_;
  Scan scan_;

  // Transition to a new state.
  void transition(State state);

  ros::Publisher cmd_vel_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber bumper_sub_;
  ros::Subscriber wheeldrop_sub_;

  // Handle odometry messages.
  void odomCallback(const nav_msgs::OdometryConstPtr &msg);

  // Handle bumper messages.
  void bumperCallback(const ca_msgs::BumperConstPtr &msg);

  // Handle wheeldrop messages.
  void wheeldropCallback(const std_msgs::EmptyConstPtr &msg);

public:
  Wander(ros::NodeHandle &nh);
  ~Wander(){};  // Suppress copy and move constructors

  // We're called at 10Hz. Publish a new velocity.
  void spinOnce();
};

} // namespace rodeobot

#endif // RODEOBOT_WANDER_H
