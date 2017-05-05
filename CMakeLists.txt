cmake_minimum_required(VERSION 2.8.3)
project(rrbot_description)

#find_package(catkin REQUIRED)
find_package(catkin REQUIRED 
  message_generation
  interactive_markers
  roscpp 
  visualization_msgs 
  tf 
  actionlib_msgs
  std_msgs
  geometry_msgs
)

##########
## MSGS ##
##########

add_message_files(DIRECTORY msg 
  FILES
  XY_position.msg
)

#add_service_files(DIRECTORY srv 
#  FILES
#)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

#catkin_package()
catkin_package(
CATKIN_DEPENDS message_runtime interactive_markers roscpp visualization_msgs tf std_msgs geometry_msgs 
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dragonfly_markers src/dragonfly_markers.cpp)
target_link_libraries(dragonfly_markers
   ${catkin_LIBRARIES}
)

## Build talker and listener
add_executable(pose_publisher src/pose_publisher.cpp)
target_link_libraries(pose_publisher
   ${catkin_LIBRARIES}
)

add_dependencies(pose_publisher beginner_tutorials_generate_messages_cpp)

add_executable(click_subscriber src/click_subscriber.cpp)
target_link_libraries(click_subscriber ${catkin_LIBRARIES})
add_dependencies(click_subscriber beginner_tutorials_generate_messages_cpp)

add_executable(pose_sub src/pose_sub.cpp)
target_link_libraries(pose_sub ${catkin_LIBRARIES})
add_dependencies(pose_sub beginner_tutorials_generate_messages_cpp)

#add_executable(basic_controls_test src/basic_controls_test.cpp)
#target_link_libraries(basic_controls_test
#   ${catkin_LIBRARIES}
#)

#add_executable(simple_marker_test src/simple_marker_test.cpp)
#target_link_libraries(simple_marker_test
#   ${catkin_LIBRARIES}
#)

#add_executable(test2_marker src/test2_marker.cpp)
#target_link_libraries(test2_marker
#   ${catkin_LIBRARIES}
#)

#############
## Install ##
#############

install(PROGRAMS
  #scripts/basic_controls_test.py
  #scripts/cube.py
  #scripts/menu.py
  #scripts/simple_marker_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS
  #simple_marker_test
  #basic_controls_test
  dragonfly_markers
  #test2_marker
  pose_publisher
  click_subscriber
  pose_sub
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY meshes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
