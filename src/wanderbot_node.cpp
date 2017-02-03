#include <rodeobot/wanderbot.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wanderbot_node");
  ros::NodeHandle nh{"~"};
  tf::TransformListener tf{nh};
  rodeobot::Wanderbot wander{nh, tf};

  ros::Rate r(10);
  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();
    wander.spinOnce();
  }

  return 0;
}
