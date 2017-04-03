# Dragonfly

# Top level is all the work I have so far about the interactive markers and the 3d model. 

## Setup:
clone repo into new repository named 'rrbot_description'

in your ROS workspace run '<workspace_name>_make

## To run the interactive marker alone:
1. roscore
2. rosrun rrbot_description test_marker
3. rosrun rrbot_description subscriber
4. rosrun rviz rviz 

#### add interactive marker component in Rviz
1. Make sure update topic is test_finger_marker/update
..* (click anywhere on rviz)
2. Click on part(s) to see the messages pop up in subscriber terminal

## To see 3D model launch:
1. roscore
2. roslaunch rrbot_description rrbot_rviz.launch

## more to come!
