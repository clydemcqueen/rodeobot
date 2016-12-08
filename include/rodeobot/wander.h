#ifndef RODEOBOT_WANDER_H
#define RODEOBOT_WANDER_H 

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include "ca_msgs/Bumper.h"

class Scan
{
private:
  std::vector<double> map_{ std::vector<double>(360) };

public:
  void init();
  void putValue(double yaw, double signal);
  double getBestAngle();
};

class DriveModel
{
private:
  bool driving_;

public:
  static const double max_v_;
  static const double accel_;

  bool driving() { return driving_; }

  void initDrive() { driving_ = true; }
  void initStop() { driving_ = false; }

  double computeLinearX(double v0, double dt);
};

class RotateModel
{
private:
  double goal_yaw_;           // Goal yaw
  bool full_circle_;          // True: rotate one full circle
  bool clockwise_;            // True: go clockwise
  bool suppress_goal_check_;  // True: suppress the goal check for now

public:
  static const double max_v_;
  static const double accel_;
  static const double epsilon_;

  bool fullCircle() { return full_circle_; }
  bool clockwise() { return clockwise_; }

  void initGoalAngle(double current_yaw, double goal_yaw);
  void initFullCircle(double current_yaw);
  double computeAngularZ(double yaw, double v0, double dt);
};

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
  nav_msgs::Odometry last_odom_;
  double last_yaw_;

  ca_msgs::Bumper last_bumper_;
  ros::Time last_bumper_time_;  // TODO use the time in the bumper header instead

  geometry_msgs::Twist last_twist_;
  ros::Time last_twist_time_;

  State state_{ State::start };
  RotateModel rotate_model_;
  DriveModel drive_model_;
  Scan scan_;
  double start_yaw_;
  void transition(State state);

  ros::Publisher cmd_vel_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber bumper_sub_;
  ros::Subscriber wheeldrop_sub_;

  void odomCallback(const nav_msgs::OdometryConstPtr &msg);
  void bumperCallback(const ca_msgs::BumperConstPtr &msg);
  void wheeldropCallback(const std_msgs::EmptyConstPtr &msg);

public:
  Wander(ros::NodeHandle &nh);
  ~Wander(){};  // Suppress copy and move constructors
  void spinOnce();
};

#endif // RODEOBOT_WANDER_H
