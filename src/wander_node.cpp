#include <rodeobot/wander.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "wander");
  ros::NodeHandle nh; // TODO use ("~") ?

  rodeobot::Wander wander{ nh };

  ros::Rate r(10);
  while (ros::ok())
  {
    // Handle messages.
    // TODO needed? is costmap on it's own thread?
    ros::spinOnce();

    // Plan and/or move 10x per second.
    wander.spinOnce();

    // Sleep for the remainder of our cycle time.
    r.sleep();

    // Did we finish in time?
    if (r.cycleTime() > ros::Duration(1/10))
      ROS_WARN("Control loop took %f seconds", r.cycleTime().toSec());
  }

  return 0;
}
