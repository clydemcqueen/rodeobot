#include <rodeobot/wanderbot.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wanderbot_node");
  ros::NodeHandle nh{"~"};
  tf::TransformListener tf{nh};
  rodeobot::Wanderbot wander{nh, tf};

  ros::Timer timer1 = nh.createTimer(ros::Duration(0.1), &rodeobot::Wanderbot::spinOnce, &wander);

  ros::spin();

  return 0;
}
