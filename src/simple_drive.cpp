#include <iostream>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

// Everything in ROS is metric: m, m/s, m/s^2.

class SimpleDrive
{
private:
  const float cruising_ {0.4};          // Cruising velocity (max from libcreate)
  const float accel_ {0.1};             // Acceleration
  const float epsilon_ {0.02};          // Target error

  float target_distance_;               // How far we want to go
  float current_distance_;              // How far we've gone so far
  geometry_msgs::Twist last_twist_;     // Last twist (velocity) we sent
  ros::Time last_time_;                 // Time of last twist message

  ros::Publisher cmd_vel_pub_;
  ros::Subscriber drive_sub_;
  ros::Subscriber odom_sub_;

  void driveCallback(const std_msgs::Float32ConstPtr& msg);
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);

  void publishCmdVel();

public:
  SimpleDrive(ros::NodeHandle& nh);
  ~SimpleDrive() {}; // This will suppress the default copy and constructors
};

// Constructor.
SimpleDrive::SimpleDrive(ros::NodeHandle& nh)
{
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  drive_sub_ = nh.subscribe("drive", 1, &SimpleDrive::driveCallback, this);
  odom_sub_ = nh.subscribe("odom", 1, &SimpleDrive::odomCallback, this);
}

// Handle drive message. This is where we get our simple instructions.
void SimpleDrive::driveCallback(const std_msgs::Float32ConstPtr& msg)
{
  std::cout << "drive=" << msg->data << std::endl;
  target_distance_ = msg->data;
  publishCmdVel();
}

// Handle odometry message.
void SimpleDrive::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
  // std::cout << "odom x=" << msg->pose.pose.position.x << " y=" << msg->pose.pose.position.y << std::endl;
  current_distance_ = msg->pose.pose.position.x;
  publishCmdVel();
}

// Publish a new velocity.
void SimpleDrive::publishCmdVel()
{
  auto now {ros::Time::now()};
  auto v0 {last_twist_.linear.x};

  geometry_msgs::Twist msg; // TODO does this default / init to 0?

  // Are we close enough?
  if (target_distance_ - current_distance_ < epsilon_)
  {
    // Full stop. Report any over/undershoot.
    if (v0 > 0.01)
      std::cout << "stopping, error=" << target_distance_ - current_distance_ << std::endl;
  }
  else
  {
    // Compute the stopping distance at our current velocity.
    auto stopping_time {v0/accel_};
    auto stopping_distance = v0 * stopping_time + 0.5 * accel_ * stopping_time * stopping_time;

    // Compute the target velocity.
    if (target_distance_ - current_distance_ < stopping_distance)
    {
      // We're decelerating.
      msg.linear.x = v0 - accel_ * (now - last_time_).toSec();
    }
    else if (v0 < cruising_)
    {
      // We're accelerating.
      msg.linear.x = v0 + accel_ * (now - last_time_).toSec();
    }
    else
    {
      // We're cruising.
      msg.linear.x = cruising_;
    }
  }

  // Publish the new velocity.
  if (v0 > 0.01 || msg.linear.x > 0.01)
    std::cout << "v0=" << v0 << " v1=" << msg.linear.x << std::endl;
  cmd_vel_pub_.publish(msg);
  last_twist_ = msg;
  last_time_ = now;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_drive");
  ros::NodeHandle nh;

  SimpleDrive simple_drive{nh};

  ros::spin();
  return 0;
}
