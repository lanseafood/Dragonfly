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

// Controller for marker A
void ControlA(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  ROS_INFO_STREAM("CONTROLLER A CALLED");
  std_msgs::String news;
  std::ostringstream s;
  s << feedback->marker_name;
  news.data = s.str();
  meshClick_pub.publish(news);
}

// Controller for marker B
void ControlB(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
  ROS_INFO_STREAM("CONTROLLER B CALLED");
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
  server.insert(InsertMarker("marker_finger", 0.0, -1.0, "package://rrbot_description/meshes/finger.stl"), &ControlA);
  server.insert(InsertMarker("marker_mount", 0.0,  1.0, "package://rrbot_description/meshes/mount.stl"), &ControlB);
  // 'commit' changes and send to all clients
  server.applyChanges();
  // start the ROS main loop
  ros::spin();
}

// #include <ros/ros.h>

// #include <interactive_markers/interactive_marker_server.h>
// #include <interactive_markers/menu_handler.h>

// #include <tf/transform_broadcaster.h>
// #include <tf/tf.h>

// #include <math.h>

// #include "std_msgs/String.h"
// #include <sstream>
// using namespace visualization_msgs;

// // %Tag(vars)%
// boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
// // boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server1;
// interactive_markers::MenuHandler menu_handler_finger;
// interactive_markers::MenuHandler menu_handler_mount;

// ros::Publisher meshClick_pub;

// // %EndTag(vars)%

// void processFeedback(
//     const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
// {
//   std_msgs::String msg;

//   std::stringstream mstring;    

//   std::ostringstream s;
//   s << "Feedback from marker '" << feedback->marker_name << "' "
//       << " / control '" << feedback->control_name << "'";
//   // mstring << s;
//   msg.data = s.str();
//   meshClick_pub.publish(msg);             

//   std::ostringstream mouse_point_ss;
//   if( feedback->mouse_point_valid )
//   {
//     mouse_point_ss << " at " << feedback->mouse_point.x
//                    << ", " << feedback->mouse_point.y
//                    << ", " << feedback->mouse_point.z
//                    << " in frame " << feedback->header.frame_id;
//   }

//   ROS_INFO_STREAM( feedback->marker_name << " is now at "
//       << feedback->pose.position.x << ", " << feedback->pose.position.y
//       << ", " << feedback->pose.position.z );
//   if ( feedback-> visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT)
//   {
//     ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
//   }
//   server->applyChanges();
// }

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "test_finger_marker");

//   // MARKER THINGS 
//   // ros::init(argc, argv, "meshClickTalker");

//   ros::NodeHandle n;

//   meshClick_pub = n.advertise<std_msgs::String>("meshClick_chatter", 1000);


//   server.reset( new interactive_markers::InteractiveMarkerServer("test_finger_marker_reset","",false) );

//   // create an interactive marker server on the topic namespace simple_marker
//   interactive_markers::InteractiveMarkerServer server("test_finger_marker");

//   // create an interactive marker for our server
//   visualization_msgs::InteractiveMarker int_marker;
//   int_marker.header.frame_id = "base_link";
//   int_marker.header.stamp=ros::Time::now();
//   int_marker.name = "finger_marker_test";
//   int_marker.description = "testing button 1";

//   visualization_msgs::InteractiveMarker int_marker_2;
//   int_marker_2.header.frame_id = "other_link";
//   int_marker_2.header.stamp=ros::Time::now();
//   int_marker_2.name = "mount_marker_test";
//   int_marker_2.description = "testing button 2";


//   //interactive menu control;
//   InteractiveMarkerControl button_control;
//   button_control.interaction_mode = InteractiveMarkerControl::BUTTON;
//   button_control.name = "button_control";
   
//   InteractiveMarkerControl button_control_2;
//   button_control.interaction_mode = InteractiveMarkerControl::BUTTON;
//   button_control.name = "button_control_2";


//   // create a grey finger marker
//   visualization_msgs::Marker finger;
//   finger.type = visualization_msgs::Marker::MESH_RESOURCE;
//   finger.mesh_resource = "package://rrbot_description/meshes/finger.stl";
//   // finger.type = visualization_msgs::Marker::CUBE;
//   finger.scale.x = .01;
//   finger.scale.y = .01;
//   finger.scale.z = .01;
//   finger.color.r = 0.5;
//   finger.color.g = 0.5;
//   finger.color.b = 0.5;
//   finger.color.a = 1.0;

//   visualization_msgs::Marker mount;
//   mount.type = visualization_msgs::Marker::MESH_RESOURCE;
//   mount.mesh_resource = "package://rrbot_description/meshes/mount.stl";
//   // mount.type = visualization_msgs::Marker::CUBE;
//   mount.scale.x = .01;
//   mount.scale.y = .01;
//   mount.scale.z = .01;
//   mount.color.r = 0.5;
//   mount.color.g = 0.5;
//   mount.color.b = 0.5;
//   mount.color.a = 1.0;

//   button_control.markers.push_back(finger);
//   button_control_2.markers.push_back(mount);
//   // menu_control.markers.push_back(mount);
//   button_control.always_visible = true;
//   button_control_2.always_visible = true;

//   // add the control to the interactive marker
//   // int_marker.controls.push_back( box_control );
//   int_marker.controls.push_back( button_control);
//   int_marker_2.controls.push_back(button_control_2);
//   // int_marker.controls.push_back(menu_control);

//   // add the interactive marker to our collection &
//   // tell the server to call processFeedback() when feedback arrives for it
//   server.insert(int_marker, &processFeedback);
//   // server.applyChanges();

//   server.insert(int_marker_2, &processFeedback);

//   // 'commit' changes and send to all clients
//   // menu_handler_finger.apply( server, int_marker.name);
//   // menu_handler_mount.apply( server, int_marker.name);
//   server.applyChanges();


//   // start the ROS main loop
//   ros::spin();
// }
// // %Tag(fullSource)%
