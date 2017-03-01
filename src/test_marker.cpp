/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


// %Tag(fullSource)%
#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

using namespace visualization_msgs;

// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
// boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server1;
interactive_markers::MenuHandler menu_handler_finger;
interactive_markers::MenuHandler menu_handler_mount;
// interactive_markers::MenuHandler menu_handler_f2;
// %EndTag(vars)%

void processFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

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

void processFeedback1(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback, boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server  )
{
  std::ostringstream s;
  s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

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
  ros::init(argc, argv, "simple_marker");
  // ros::init(argc, argv, "tester_marker");

  server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );
  // server1.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );

  menu_handler_finger.insert( "finger Entry", &processFeedback );
  menu_handler_mount.insert( "mount Entry", &processFeedback );
  // menu_handler_mount.insert( "mount Entry", &processFeedback1, server1 );
  // menu_handler_f2.insert("f2 entry", &processFeedback);

  // interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Submenu" );
  // menu_handler.insert( sub_menu_handle, "First Entry", &processFeedback );
  // menu_handler.insert( sub_menu_handle, "Second Entry", &processFeedback );

  // create an interactive marker server on the topic namespace simple_marker
  interactive_markers::InteractiveMarkerServer server("simple_marker");
  // interactive_markers::InteractiveMarkerServer server("tester_marker");

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_link";
  int_marker.header.stamp=ros::Time::now();
  int_marker.name = "menu_marker_test";
  int_marker.description = "Testing_interactive_marker";
  // //tester
  visualization_msgs::InteractiveMarker int_marker_mount;
  int_marker_mount.header.frame_id = "mount_link";
  int_marker_mount.header.stamp=ros::Time::now();
  int_marker_mount.name = "mount_menu_test";
  int_marker_mount.description = "Testing_second_marker";

  //interactive menu control;
  InteractiveMarkerControl menu_control;
  menu_control.interaction_mode = InteractiveMarkerControl::MENU;
  menu_control.name = "menu_control";
  
  InteractiveMarkerControl menu_control_mount;
  menu_control_mount.interaction_mode = InteractiveMarkerControl::MENU;
  menu_control_mount.name = "menu_control_mount";



  // create a grey finger marker
  visualization_msgs::Marker finger;
  finger.type = visualization_msgs::Marker::MESH_RESOURCE;
  finger.mesh_resource = "package://rrbot_description/meshes/finger.stl";
  // finger.type = visualization_msgs::Marker::CUBE;
  finger.scale.x = .01;
  finger.scale.y = .01;
  finger.scale.z = .01;
  finger.color.r = 0.5;
  finger.color.g = 0.5;
  finger.color.b = 0.5;
  finger.color.a = 1.0;

    // create a grey mount marker
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

  //   // create a grey f2 marker
  // visualization_msgs::Marker f2;
  // f2.type = visualization_msgs::Marker::MESH_RESOURCE;
  // f2.mesh_resource = "package://rrbot_description/meshes/F2.stl";
  // // mount.type = visualization_msgs::Marker::CUBE;
  // f2.scale.x = .1;
  // f2.scale.y = .1;
  // f2.scale.z = .1;
  // f2.color.r = 0.5;
  // f2.color.g = 0.5;
  // f2.color.b = 0.5;
  // f2.color.a = 1.0;

  // create a non-interactive control which contains the box
  // visualization_msgs::InteractiveMarkerControl box_control;
  // box_control.always_visible = true;
  // box_control.markers.push_back( finger );
  // box_control.markers.push_back(mount);
  // box_control.markers.push_back(f2);


  menu_control.markers.push_back(finger);
  menu_control.markers.push_back(mount);
  // menu_control.markers.push_back(f2);
  menu_control.always_visible = true;
  menu_control.always_visible = true;

  // add the control to the interactive marker
  // int_marker.controls.push_back( box_control );
  int_marker.controls.push_back( menu_control);
  int_marker.controls.push_back(menu_control);

  // // create a control which will move the box
  // // this control does not contain any markers,
  // // which will cause RViz to insert two arrows
  // visualization_msgs::InteractiveMarkerControl rotate_control;
  // rotate_control.name = "move_x";
  // rotate_control.interaction_mode =
  //     visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

  // // add the control to the interactive marker
  // int_marker.controls.push_back(rotate_control);

  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server.insert(int_marker, &processFeedback);
  // server.insert(int_marker_mount, &processFeedback);
  // 'commit' changes and send to all clients
  menu_handler_finger.apply( server, int_marker.name);
  menu_handler_mount.apply( server, int_marker.name);
  // menu_handler_f2.apply( server, int_marker.name);
  server.applyChanges();


  // start the ROS main loop
  ros::spin();
}
// %Tag(fullSource)%
