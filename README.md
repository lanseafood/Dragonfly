# Dragonfly

# All the work I have so far about the interactive markers and the 3d model. 
This package launches rviz with two example interactive markers where users can click on the markers, change their colors, and display their names. Given proper input to the publisher.cpp file, users can also tell these markers to move to specific positions. 

## Setup:
clone repo into new repository named 'dragonfly_interactive_markers'

in your ROS workspace run '<workspace_name>_make

## To Run:
1. roslaunch rrbot_description dragonfly.launch
2. To see the interactive markers move, run in a separate terminal: rosrun [package_name] pose_publisher

## About pub and sub
1. POSE

*pose_publisher*
publishes at .5 HZ different positions for markers for user defined positions
run after the launch file with: rosrun rrbot_description pose_publisher

*pose_sub*
a test subscriber that outputs the name, position, and orientation of the marker to move
run with: rosrun rrbot_description pose_sub


2. CLICK

*click_subscriber*
publishes to a terminal the names of the markers that are clicked.
run with: rosrun rrbot_description click_subscriber


## About the Interactive Marker
The dragonfly_markers.cpp file runs the interactive marker server and allows for the user to click, change color, and move markers according to Pose data. 

## About the XY_Position.msg
This message passes through the name of the marker to move, the position to move to, and the orientation to change to. 
1. geometry_msgs/Point position

position is (x, y, z)

2. geometry_msgs/Quaternion orientation

orientation is (x, y, z, w)

3. string name



## more to come!
