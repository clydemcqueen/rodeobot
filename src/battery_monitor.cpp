#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>

class BatteryMonitor
{
private:
  int voltage_;
  int charge_ratio_;
  ros::Publisher set_ascii_pub_;

  void voltageCallback(const std_msgs::Float32ConstPtr& msg);
  void chargeRatioCallback(const std_msgs::Float32ConstPtr& msg);
  void updateDisplay();

public:
  BatteryMonitor(ros::NodeHandle& nh);
};

// Trim a float to a number between 0 and 99.
int trimFloat(float f)
{
  if (f < 0.0)
    return 0;
  if (f > 99.9)
    return 99;
  return static_cast<int>(f);
}

// Constructor.
BatteryMonitor::BatteryMonitor(ros::NodeHandle& nh) : voltage_(0), charge_ratio_(0)
{
  set_ascii_pub_ = nh.advertise<std_msgs::UInt8MultiArray>("set_ascii", 1);

  nh.subscribe("battery/voltage", 1, &BatteryMonitor::voltageCallback, this);
  nh.subscribe("battery/charge_ratio", 1, &BatteryMonitor::chargeRatioCallback, this);
}

// Handle voltage message.
void BatteryMonitor::voltageCallback(const std_msgs::Float32ConstPtr& msg)
{
  const int new_voltage = trimFloat(msg->data);
  if (new_voltage != voltage_)
  {
    voltage_ = new_voltage;
    updateDisplay();
  }
}

// Handle charge ratio message.
void BatteryMonitor::chargeRatioCallback(const std_msgs::Float32ConstPtr& msg)
{
  const int new_charge_ratio = trimFloat(msg->data);
  if (new_charge_ratio != charge_ratio_)
  {
    charge_ratio_ = new_charge_ratio;
    updateDisplay();
  }
}

// Update the display: 2 digits for voltage, 2 digits for charge ratio.
void BatteryMonitor::updateDisplay()
{
  ROS_INFO_STREAM("voltage=" << voltage_ << " charge=" << charge_ratio_);

  std_msgs::UInt8MultiArray msg;
  msg.data.clear();

  msg.data.push_back(voltage_ / 10);
  msg.data.push_back(voltage_ % 10);
  msg.data.push_back(charge_ratio_ / 10);
  msg.data.push_back(charge_ratio_ % 10);

  set_ascii_pub_.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "battery_monitor");
  ros::NodeHandle nh;

  BatteryMonitor battery_monitor{nh};

  ros::spin();
  return 0;
}
