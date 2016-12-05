#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>

class BatteryMonitor
{
private:
  int voltage_;
  int charge_ratio_;
  bool is_passive_;

  ros::Publisher set_ascii_pub_;
  ros::Publisher dock_pub_;
  ros::Publisher undock_pub_;

  ros::Subscriber voltage_sub_;
  ros::Subscriber charge_ratio_sub_;
  ros::Subscriber clean_button_sub_;

  void display(int d);

  void voltageCallback(const std_msgs::Float32ConstPtr& msg);
  void chargeRatioCallback(const std_msgs::Float32ConstPtr& msg);
  void cleanButtonCallback(const std_msgs::EmptyConstPtr& msg);

public:
  BatteryMonitor(ros::NodeHandle& nh);
  ~BatteryMonitor() {}; // This will suppress the default copy and constructors

  void displayVoltage() { display(voltage_); }
  void displayChargeRatio() { display(charge_ratio_); }
};

// Take a float of the form xx.yy and generate an int of the form xxyy.
int format(float f)
{
  if (f < 0.0)
    return 0;
  if (f > 99.99)
    return 9999;
  return static_cast<int>(f * 100);
}

// Write a 4-digit integer to the display.
void BatteryMonitor::display(int i)
{
  std_msgs::UInt8MultiArray msg;
  msg.data.clear();

  for (int m = 1000; m > 0; m /= 10)
  {
    int d = i / m;
    msg.data.push_back(d + 48);
    i -= d * m;
  }

  set_ascii_pub_.publish(msg);
}

// Constructor.
BatteryMonitor::BatteryMonitor(ros::NodeHandle& nh) : voltage_(0), charge_ratio_(0)
{
  std::cout << "Rodeobot awake!" << std::endl;

  // Set up publishers.
  set_ascii_pub_ = nh.advertise<std_msgs::UInt8MultiArray>("set_ascii", 1);
  dock_pub_ = nh.advertise<std_msgs::Empty>("dock", 1);
  undock_pub_ = nh.advertise<std_msgs::Empty>("undock", 1);

  // Subscriptions stop when Subscriber objects go out of scope -- so save them.
  voltage_sub_ = nh.subscribe("battery/voltage", 1, &BatteryMonitor::voltageCallback, this);
  charge_ratio_sub_ = nh.subscribe("battery/charge_ratio", 1, &BatteryMonitor::chargeRatioCallback, this);
  clean_button_sub_ = nh.subscribe("clean_button", 1, &BatteryMonitor::cleanButtonCallback, this);
}

// Handle voltage message. Range should be something like [0.0, ~16.0].
void BatteryMonitor::voltageCallback(const std_msgs::Float32ConstPtr& msg)
{
  auto new_voltage = format(msg->data);
  if (new_voltage != voltage_)
    voltage_ = new_voltage;
}

// Handle charge ratio message. Range should be something like [0.0, 1.0).
void BatteryMonitor::chargeRatioCallback(const std_msgs::Float32ConstPtr& msg)
{
  auto new_charge_ratio = format(msg->data * 100);
  if (new_charge_ratio != charge_ratio_)
    charge_ratio_ = new_charge_ratio;
}

// Handle clean button press.
void BatteryMonitor::cleanButtonCallback(const std_msgs::EmptyConstPtr& msg)
{
  if (is_passive_)
  {
    std::cout << "Clean button pressed; undocking" << std::endl;

    std_msgs::Empty msg2;
    undock_pub_.publish(msg2);
    is_passive_ = false;
  }
  else
  {
    std::cout << "Clean button pressed; docking" << std::endl;

    std_msgs::Empty msg2;
    dock_pub_.publish(msg2);
    is_passive_ = true;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "battery_monitor");
  ros::NodeHandle nh;

  BatteryMonitor battery_monitor{nh};

  // Alternate between displaying voltage and charge_ratio.
  ros::Rate r(0.5);
  while (ros::ok())
  {
    ros::spinOnce();
    battery_monitor.displayVoltage();
    r.sleep();

    if (ros::ok())
    {
      ros::spinOnce();
      battery_monitor.displayChargeRatio();
      r.sleep();
    }
  }

  return 0;
}
