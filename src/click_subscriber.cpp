#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
/**
 * This subscriber listens to the click publisher in interesting_marker.cpp and publishes the clicked info.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I was clicked and my name is : [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("meshClick_chatter", 1000, chatterCallback);

  ros::spin();

  return 0;
}