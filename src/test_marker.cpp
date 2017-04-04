#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

#include "std_msgs/String.h"
#include <sstream>
#include <string>

using namespace visualization_msgs;
ros::Publisher meshClick_pub;

// Clickable for markers
void onClick(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  ROS_INFO_STREAM("CONTROLLER " + feedback->marker_name + " CLICKED ");
  std_msgs::String news;
  std::ostringstream s;
  s << feedback->marker_name;
  news.data = s.str();
  meshClick_pub.publish(news);
}


// Function to insert a marker with a specific name
visualization_msgs::InteractiveMarker InsertMarker(std::string const& name, float x, float y, std::string const stl) {
  // Create the interactive marker
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.header.stamp = ros::Time::now();
  int_marker.name = name;
  int_marker.description = "Simple button Control";
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
  mesh_marker.color.r = 0.5;
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

int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "clickable_markers_server");
  // create an interactive marker server on the topic namespace simple_marker
  interactive_markers::InteractiveMarkerServer server("IM_example_marker");

  ros::NodeHandle n;

  meshClick_pub = n.advertise<std_msgs::String>("meshClick_chatter", 1000);
 
  // Use two different controller types
  server.insert(InsertMarker("marker_finger", 0.0, -1.0, "package://rrbot_description/meshes/finger.stl"), &onClick);
  server.insert(InsertMarker("marker_mount", 0.0,  1.0, "package://rrbot_description/meshes/mount.stl"), &onClick);
  // 'commit' changes and send to all clients
  server.applyChanges();
  // start the ROS main loop
  ros::spin();
}
