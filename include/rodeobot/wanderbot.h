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
  planning,       // We're planning our next goal.
  driving,        // We're driving toward our goal.
  emergency_stop  // We've stopped completely, and we're done.
};

class Wanderbot
{
private:
  ros::NodeHandle &nh_;
  tf::TransformListener &tf_;
  costmap_2d::Costmap2DROS *costmap_;
  dwa_local_planner::DWAPlannerROS *driver_;
  std::vector<geometry_msgs::PoseStamped> plan_;

  // Used for deciding when to start.
#if 0
  bool saw_odom_{false}; // TODO?
#endif
  bool saw_bumper_{false};

#if 0
  // Last yaw we saw.
  double last_yaw_{0.0}; // TODO?
#endif

#if 0
  // Last twist we sent.
  geometry_msgs::Twist last_twist_; // TODO?
  ros::Time last_twist_time_; // TODO?
#endif

  // Internal state.
  State state_{State::start};
#if 0
  double start_yaw_{0.0}; // TODO?
#endif

  // Transition to a new state.
  void transition(State state);

  ros::Publisher cmd_vel_pub_;
#if 0
  ros::Subscriber odom_sub_; // TODO?
#endif
  ros::Subscriber bumper_sub_;
  ros::Subscriber wheeldrop_sub_;

#if 0
  // Handle odometry messages.
  void odomCallback(const nav_msgs::OdometryConstPtr &msg); // TODO?
#endif

  // Handle bumper messages.
  void bumperCallback(const ca_msgs::BumperConstPtr &msg);

  // Handle wheeldrop messages.
  void wheeldropCallback(const std_msgs::EmptyConstPtr &msg);

public:
  explicit Wanderbot(ros::NodeHandle &nh, tf::TransformListener &tf);
  ~Wanderbot();

  void spinOnce();
};

} // namespace rodeobot

#endif // RODEOBOT_WANDER_H
