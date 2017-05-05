// this publisher must be given the name, position, and oritentation data to publish it to Rviz and the corresponding interactive markerse. 

#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

#include "std_msgs/Int64.h"
#include "rrbot_description/XY_position.h"
#include <tr1/tuple>

using namespace visualization_msgs;

ros::Publisher meshClick_pub;

// initializing the talker
std::tr1::tuple <std::string, geometry_msgs::Point, geometry_msgs::Quaternion> makeTuple(std::string name, int pX, int pY, int pZ, int oX, int oY, int oZ, int oW)
{
  // making the point position 
  geometry_msgs::Point p;
  p.x = pX;
  p.y = pY;
  p.z = pZ;
  //making the orientation
  geometry_msgs::Quaternion q;
  q.x = oX;
  q.y = oY;
  q.z = oZ;
  q.w = oW;
  //assembling the tuple
  std::tr1::tuple <std::string, geometry_msgs::Point, geometry_msgs::Quaternion> tp (name, p, q);
  return tp;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fakePoses");

  ros::NodeHandle n;
  //make publisher
  meshClick_pub = n.advertise<rrbot_description::XY_position>("fakePoses_chatter", 1000);
  ros::Rate loop_rate(0.5); //a slow ish speed

  //vector holds the name, the pose, and the orientation. 
  std::vector<std::tr1::tuple<std::string, geometry_msgs::Point, geometry_msgs::Quaternion> > movement;

  //some examples of movement
  movement.push_back( makeTuple("marker_finger", 1, 0, 1, 0, 0, 0, 0));
  movement.push_back( makeTuple("marker_mount", -1, 1, 0, 0, 0, 0, 0));
  movement.push_back( makeTuple("marker_finger", 2, 0, -1, 0, 0, 0, 0));
  movement.push_back( makeTuple("marker_mount", -2, -1, 0, 0, 0, 0, 0));
  movement.push_back( makeTuple("marker_finger", 0, 0, 2, 0, 0, 0, 0));
  movement.push_back( makeTuple("marker_mount", 0, 2, 0, 0, 0, 0, 0));

  for (int i = 0; i <movement.size(); i++)
  {
    //create msg
    rrbot_description::XY_position msg;

    //append position, orientation, and name to the msg
    msg.position = std::tr1::get<1>(movement[i]);
    msg.orientation = std::tr1::get<2>(movement[i]);
    msg.name = std::tr1::get<0>(movement[i]);

    //publish the msg
    meshClick_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  // start the ROS main loop
  ros::spin();
}
// %Tag(fullSource)%
