
#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

#include "std_msgs/String.h"
#include <sstream>
using namespace visualization_msgs;

// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
// boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server1;
interactive_markers::MenuHandler menu_handler_finger;
interactive_markers::MenuHandler menu_handler_mount;

ros::Publisher meshClick_pub;

// %EndTag(vars)%

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std_msgs::String msg;

  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";
  msg.data = s.str();
  meshClick_pub.publish(msg);             

  std::ostringstream mouse_point_ss;
  if( feedback->mouse_point_valid )
  {
    mouse_point_ss << " at " << feedback->mouse_point.x
                   << ", " << feedback->mouse_point.y
                   << ", " << feedback->mouse_point.z
                   << " in frame " << feedback->header.frame_id;
  }

  ROS_INFO_STREAM( feedback->marker_name << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z );
  if ( feedback-> visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT)
  {
    ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
  }
  server->applyChanges();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_mount_marker");

  // MARKER THINGS 
  // ros::init(argc, argv, "meshClickTalker");

  ros::NodeHandle n;

  meshClick_pub = n.advertise<std_msgs::String>("meshClick_chatter", 1000);


  server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls_2","",false) );

  // create an interactive marker server on the topic namespace simple_marker
  interactive_markers::InteractiveMarkerServer server("test_mount_marker");

  // create an interactive marker for our server

  visualization_msgs::InteractiveMarker int_marker_2;
  int_marker_2.header.frame_id = "other_link";
  int_marker_2.header.stamp=ros::Time::now();
  int_marker_2.name = "mount_marker_test";
  int_marker_2.description = "testing button 2";


  //interactive menu control;
  InteractiveMarkerControl button_control2;
  button_control2.interaction_mode = InteractiveMarkerControl::BUTTON;
  button_control2.name = "button_control2";
   


  visualization_msgs::Marker mount;
  mount.type = visualization_msgs::Marker::MESH_RESOURCE;
  mount.mesh_resource = "package://rrbot_description/meshes/mount.stl";
  // mount.type = visualization_msgs::Marker::CUBE;
  mount.scale.x = .01;
  mount.scale.y = .01;
  mount.scale.z = .01;
  mount.color.r = 0.5;
  mount.color.g = 0.5;
  mount.color.b = 0.5;
  mount.color.a = 1.0;

  button_control2.markers.push_back(mount);
  // menu_control.markers.push_back(mount);
  button_control2.always_visible = true;
  // menu_control.always_visible = true;

  // add the control to the interactive marker
  // int_marker.controls.push_back( box_control );
  int_marker_2.controls.push_back(button_control2);
  // int_marker.controls.push_back(menu_control);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it

  server.insert(int_marker_2, &processFeedback);

  // 'commit' changes and send to all clients
  // menu_handler_finger.apply( server, int_marker.name);
  // menu_handler_mount.apply( server, int_marker.name);
  server.applyChanges();


  // start the ROS main loop
  ros::spin();
}
// %Tag(fullSource)%
