#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>

class BatteryMonitor
{
private:
  int voltage_;
  int charge_ratio_;
  ros::Publisher set_ascii_pub_;
  ros::Subscriber voltage_sub_;
  ros::Subscriber charge_ratio_sub_;

  void voltageCallback(const std_msgs::Float32ConstPtr& msg);
  void chargeRatioCallback(const std_msgs::Float32ConstPtr& msg);
  void updateDisplay();

public:
  BatteryMonitor(ros::NodeHandle& nh);
  ~BatteryMonitor() {}; // This will suppress the default copy and constructors
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
  std::cout << "Rodeobot awake!" << std::endl;

  set_ascii_pub_ = nh.advertise<std_msgs::UInt8MultiArray>("set_ascii", 1);

  // Subscriptions stop when Subscriber objects go out of scope -- so save them.
  voltage_sub_ = nh.subscribe("battery/voltage", 1, &BatteryMonitor::voltageCallback, this);
  charge_ratio_sub_ = nh.subscribe("battery/charge_ratio", 1, &BatteryMonitor::chargeRatioCallback, this);
}

// Handle voltage message. Range should be something like [0.0, ~16.0].
void BatteryMonitor::voltageCallback(const std_msgs::Float32ConstPtr& msg)
{
  const int new_voltage = trimFloat(msg->data);
  if (new_voltage != voltage_)
  {
    voltage_ = new_voltage;
    updateDisplay();
  }
}

// Handle charge ratio message. Range should be something like [0.0, 1.0].
void BatteryMonitor::chargeRatioCallback(const std_msgs::Float32ConstPtr& msg)
{
  const int new_charge_ratio = trimFloat(msg->data * 100);
  if (new_charge_ratio != charge_ratio_)
  {
    charge_ratio_ = new_charge_ratio;
    updateDisplay();
  }
}

// Update the display: 2 digits for voltage, 2 digits for charge ratio.
void BatteryMonitor::updateDisplay()
{
  std::cout << "voltage=" << voltage_ << " charge=" << charge_ratio_ << std::endl;

  std_msgs::UInt8MultiArray msg;
  msg.data.clear();

  msg.data.push_back(voltage_ / 10 + 48);
  msg.data.push_back(voltage_ % 10 + 48);
  msg.data.push_back(charge_ratio_ / 10 + 48);
  msg.data.push_back(charge_ratio_ % 10 + 48);

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
