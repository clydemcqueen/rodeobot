#ifndef RODEOBOT_W3_H
#define RODEOBOT_W3_H 

#include <mutex>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
//#include <visualization_msgs/Marker.h>

#include "ca_msgs/Bumper.h"

#define SPIN_RATE 10

namespace rodeobot {

enum class Direction
{
  none,
  cw,   // Clockwise (negative)
  ccw   // Counter-clockwise (positive)
};

enum class State
{
  start,          // We're just starting out
  drive,          // We're cruising
  recover,        // We're stuck, and recovering
  emergency_stop  // We've stopped completely, and we're done
};

class Wander
{
private:
  ros::NodeHandle &nh_;
  tf::TransformListener &tf_;

  bool good_scan_;                  // We've seen at least one laser scan
  sensor_msgs::LaserScan scan_;     // Most recent laser scan
  std::mutex scan_mutex_;           // Mutex for writing to scan_

  State state_;                     // Current state (drive, recover)
  Direction dir_{Direction::none};  // Current direction (drive, recover)
  double x_prev_v_{0.0};            // Previous forward velocity (drive)
  double r_prev_v_{0.0};            // Previous rotational velocity (drive, recover)

  ros::Subscriber laser_sub_;
  ros::Subscriber bumper_sub_;
  ros::Subscriber wheeldrop_sub_;

  ros::Publisher cmd_vel_pub_;
  ros::Publisher laser_pub_;

  void laserCallback(const sensor_msgs::LaserScanConstPtr &msg);
  void bumperCallback(const ca_msgs::BumperConstPtr &msg);
  void wheeldropCallback(const std_msgs::EmptyConstPtr &msg);

  void emergencyStop();
  void driveMotion(int arc_start, int arc_width, double &x_target_v, double &r_target_v);
  void recoverMotion(int arc_start, int arc_width, double &x_target_v, double &r_target_v);

  // Configurable parameters.
  double x_min_v_{0.1};             // Linear min velocity (m/s)
  double x_max_v_{0.3};             // Linear max velocity (m/s)
  double x_accel_{0.3};             // Linear acceleration (m/s/s)
  double r_min_v_{0.1};             // Rotational min velocity (r/s)
  double r_max_v_{0.4};             // Rotational max velocity (r/s)
  double r_accel_{0.4};             // Rotational acceleration (r/s/s)
  double r_epsilon_{0.05};          // |value| < epsilon is equiv to zero (r/s)
  double scan_width_{0.5};          // Minimum free space to move forward (m)
  double scan_horizon_{3.0};        // Our scanning horizon; ignore stuff further out (m)

public:
  explicit Wander(ros::NodeHandle &nh, tf::TransformListener &tf);
  ~Wander() {}; // Suppress default copy and move constructors

  void spinOnce(const ros::TimerEvent &event);
};

} // namespace rodeobot

#endif // RODEOBOT_W3_H
