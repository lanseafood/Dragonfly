# Dragonfly

Top level is all the work I have so far about the interactive markers and the 3d model. 

Setup:
clone repo into new repository named 'rrbot_description' (or whatever you would like)
in your ROS workspace run '<workspace_name>_make

To run the interactive marker alone:
roscore
rosrun rrbot_description simple_marker_test
rosrun rviz rviz

Add interactive Marker component
make sure update topic is simple_marker/update
(click anywhere on rviz)
Move the box around with arrows
click on box to see menu display

To see 3d model launch:
roscore
roslaunch rrbot_description rrbot_rviz.launch

more to come!
