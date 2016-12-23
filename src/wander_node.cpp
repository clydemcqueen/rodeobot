#include <rodeobot/wander.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wander");
  ros::NodeHandle nh; // TODO use ("~") ?

  rodeobot::Wander wander{ nh };

  ros::Rate r(10);
  while (ros::ok())
  {
    r.sleep();
    ros::spinOnce();
    wander.spinOnce();
  }

  return 0;
}
