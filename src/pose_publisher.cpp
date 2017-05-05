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

#include "std_msgs/Int64.h"
#include "rrbot_description/XY_position.h"
#include <tr1/tuple>

using namespace visualization_msgs;

// %Tag(vars)%

ros::Publisher meshClick_pub;
// %EndTag(vars)%

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
