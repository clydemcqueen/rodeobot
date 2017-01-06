#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "costmap_node");

	// ~/costmap means: private to node (inside node), in "costmap:" section
  ros::NodeHandle private_nh("~/costmap");
  ros::NodeHandle g_nh;

  // get our tf prefix
  ros::NodeHandle prefix_nh;
  std::string tf_prefix = tf::getPrefixParam(prefix_nh);

  ROS_INFO("tf_prefix \"%s\"", tf_prefix.c_str());

  std::string global_frame_;
  std::string robot_base_frame_;

  // get two frames
  private_nh.param("global_frame", global_frame_, std::string("/map"));
  private_nh.param("robot_base_frame", robot_base_frame_, std::string("base_link"));

  ROS_INFO("global_frame \"%s\"", global_frame_.c_str());
  ROS_INFO("robot_base_frame \"%s\"", robot_base_frame_.c_str());

  // make sure that we set the frames appropriately based on the tf_prefix
  global_frame_ = tf::resolve(tf_prefix, global_frame_);
  robot_base_frame_ = tf::resolve(tf_prefix, robot_base_frame_);

  ROS_INFO("global_frame \"%s\"", global_frame_.c_str());
  ROS_INFO("robot_base_frame \"%s\"", robot_base_frame_.c_str());

  return 0;
}
