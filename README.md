# Dragonfly

# Top level is all the work I have so far about the interactive markers and the 3d model. 

## Setup:
clone repo into new repository named 'rrbot_description' (or whatever you would like)

in your ROS workspace run '<workspace_name>_make

## To run the interactive marker alone:
1. roscore
2. rosrun rrbot_description simple_marker_test
3. rosrun rviz rviz

#### add interactive marker component in Rviz
1. Make sure update topic is simple_marker/update
..* (click anywhere on rviz)
2. Move the box around with arrows
3. Click on box to see menu display

## To see 3D model launch:
1. roscore
2. roslaunch rrbot_description rrbot_rviz.launch

## more to come!
