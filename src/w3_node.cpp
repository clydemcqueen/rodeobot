#include <rodeobot/w3.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wander");
  ros::NodeHandle nh{"~"};
  tf::TransformListener tf{nh};
  rodeobot::Wander wander{nh, tf};

  ros::Timer t = nh.createTimer(ros::Duration(1.0 / SPIN_RATE), &rodeobot::Wander::spinOnce, &wander);

  ros::spin();

  return 0;
}
