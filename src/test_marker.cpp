#include <ros/ros.h>

#include "../../../interactive_markers/include/interactive_markers/interactive_marker_server.h"
#include "../../../interactive_markers/include/interactive_markers/menu_handler.h"

#include <utility>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <typeinfo>
#include <math.h>

#include "std_msgs/String.h"
#include <sstream>
#include <string>
#include "std_msgs/Int64.h"
#include "rrbot_description/XY_position.h"
#include <tr1/tuple>

using namespace visualization_msgs;

ros::Publisher meshClick_pub;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
//map of ID's of markers and color to switch between when clicked
std::map<std::string, std::pair<int, std::string> > markers_color_map;

// Function to insert a marker with a specific name
const visualization_msgs::InteractiveMarker MakeMarker(std::string const& name, float color, std::string stl, std::string des) {
  // Create the interactive marker
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.header.stamp = ros::Time::now();
  int_marker.name = name;
  int_marker.description = des;
  // int_marker.pose.position.x = x;
  // int_marker.pose.position.y = y;
  int_marker.scale = 1.0;
 
//Create a mesh marker
  visualization_msgs::Marker mesh_marker;
  mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  mesh_marker.mesh_resource = stl;
  mesh_marker.scale.x = .01;
  mesh_marker.scale.y = .01;
  mesh_marker.scale.z = .01;
  mesh_marker.color.r = color;
  mesh_marker.color.g = 0.5;
  mesh_marker.color.b = 0.5;
  mesh_marker.color.a = 1.0;

  // Create the button control
  visualization_msgs::InteractiveMarkerControl button_control;
  button_control.name = name + "_control";
  button_control.interaction_mode =
    visualization_msgs::InteractiveMarkerControl::BUTTON;
  button_control.markers.push_back(mesh_marker);
  button_control.always_visible= true;

  // add the control to the interactive marker
  int_marker.controls.push_back(button_control);
  // return the new marker
  return int_marker;
}

// Clickable for markers
void onClick(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {

  std::cout << *feedback << std::endl;
  //std::cout << feedback->pose.position << std::endl;
  //std::cout << typeid(feedback->pose.position).name() << std::endl;

  //getting marker name
  std::string name = feedback->marker_name;

  //outputting to cout
  ROS_INFO_STREAM("CONTROLLER " + name + " CLICKED ");

  //publishing to newsfeed
  std_msgs::String news;
  std::ostringstream s;
  s << name;
  news.data = s.str();
  meshClick_pub.publish(news);

  //erasing old marker
  server->erase(name);
  std::cout << "erased old one" << std::endl;


  //checking color to change to
  float color;
  std::string d;
  int flip = markers_color_map[name].first;
  if (flip == 0)
  {
    color = 255.0;
    d = name;
    markers_color_map[name].first = 1;

  }
  else
  {
    color = 0.5;
    d = "";
    markers_color_map[name].first = 0;
  }


  //getting stl
  std::string stl = markers_color_map[name].second;
  std::cout << name << color << stl << std::endl;
  //replacing IM
  visualization_msgs::InteractiveMarker int_marker = MakeMarker(name, color, stl, d);
  
  server->insert(int_marker);


  server->applyChanges();

}
//aligning the marker to position
void alignMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback  )
{
  //geting new position
  geometry_msgs::Pose pose = feedback->pose;

  pose.position.x = round(pose.position.x-0.5)+0.5;
  pose.position.y = round(pose.position.y-0.5)+0.5;

  server->setPose(feedback->marker_name, pose);

  server->applyChanges();
}

//update Pose given data points
//msg is formate of Point (position), Quarternion (orientation), and Name (of marker to move)
void poseCallback(const rrbot_description::XY_position::ConstPtr& msg)
{
  //Create pose object
  geometry_msgs::Pose pose;

  //set Pose position from msg
  pose.position = msg->position;
  
  //round pose positions to look nicer
  pose.position.x = round(pose.position.x-0.5)+0.5;
  pose.position.y = round(pose.position.y-0.5)+0.5;

  //set Pose orientation from msg
  pose.orientation = msg->orientation;

  server->setPose(msg->name, pose);

  server->applyChanges();
}

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "clickable_markers_server");

  //initialize node and subscriber
  ros::NodeHandle pose_node;
  ros::Subscriber pose_sub = pose_node.subscribe("fakePoses_chatter", 1000, poseCallback);

  // create an interactive marker server on the topic namespace simple_marker
  ros::NodeHandle n;
  server = boost::shared_ptr<interactive_markers::InteractiveMarkerServer>(new interactive_markers::InteractiveMarkerServer ("IM_example_marker"));
  
  meshClick_pub = n.advertise<std_msgs::String>("meshClick_chatter", 1000);

  //make pairs
  std::pair<int, std::string> finger_pair (0,"package://rrbot_description/meshes/finger.stl");
  std::pair<int, std::string> mount_pair (0,"package://rrbot_description/meshes/mount.stl");

  //make markers for each part
  visualization_msgs::InteractiveMarker int_marker_1 = MakeMarker("marker_finger" , 0.5, finger_pair.second, "");
  visualization_msgs::InteractiveMarker int_marker_2 = MakeMarker("marker_mount", 0.5, mount_pair.second, "");

    // Use two different controller types
  server->insert(int_marker_1, &onClick);
  server->insert(int_marker_2, &onClick);

  //for aligning the marker 
  // server->setCallback(int_marker_1.name, &alignMarker, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE );
  // server->setCallback(int_marker_2.name, &alignMarker, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE );

  //update Map
  markers_color_map["marker_finger"] =  finger_pair;
  markers_color_map["marker_mount"] =  mount_pair;

  // 'commit' changes and send to all clients
  server->applyChanges();

  // start the ROS main loop
  ros::spin();
}
