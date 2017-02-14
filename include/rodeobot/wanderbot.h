#ifndef RODEOBOT_WANDER_H
#define RODEOBOT_WANDER_H 

#include <ros/ros.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>

#include "ca_msgs/Bumper.h"

namespace rodeobot {

enum class State
{
  start,          // We're just starting out.
  plan,           // We're planning our next goal.
  drive,          // We're driving toward our goal.
  recover,        // We're stuck, and trying to get unstuck.
  emergency_stop  // We've stopped completely, and we're done.
};

class Wanderbot
{
private:
  ros::NodeHandle &nh_;
  tf::TransformListener &tf_;
  costmap_2d::Costmap2DROS *costmap_ros_;
  dwa_local_planner::DWAPlannerROS *driver_;
  std::vector<geometry_msgs::PoseStamped> plan_;

  // Internal state.
  State state_;
  ros::Time last_plan_time_;

  void emergencyStop();

  ros::Publisher cmd_vel_pub_;
  ros::Subscriber bumper_sub_;
  ros::Subscriber wheeldrop_sub_;

  // Handle bumper messages.
  void bumperCallback(const ca_msgs::BumperConstPtr &msg);

  // Handle wheeldrop messages.
  void wheeldropCallback(const std_msgs::EmptyConstPtr &msg);

public:
  explicit Wanderbot(ros::NodeHandle &nh, tf::TransformListener &tf);
  ~Wanderbot();

  void spinOnce(const ros::TimerEvent &event);
};

} // namespace rodeobot

#endif // RODEOBOT_WANDER_H
