cmake_minimum_required(VERSION 2.8.3)
project(rrbot_description)

#find_package(catkin REQUIRED)
find_package(catkin REQUIRED 
  interactive_markers
  message_generation
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

add_service_files(DIRECTORY srv 
  FILES
  GetPartName.srv
)

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

add_executable(simple_marker_test src/simple_marker_test.cpp)
target_link_libraries(simple_marker_test
   ${catkin_LIBRARIES}
)

add_executable(test_marker src/test_marker.cpp)
target_link_libraries(test_marker
   ${catkin_LIBRARIES}
)

add_executable(test2_marker src/test2_marker.cpp)
target_link_libraries(test2_marker
   ${catkin_LIBRARIES}
)


## Build talker and listener
add_executable(publisher src/publisher.cpp)
target_link_libraries(publisher
   ${catkin_LIBRARIES}
)

add_dependencies(publisher beginner_tutorials_generate_messages_cpp)

add_executable(subscriber src/subscriber.cpp)
target_link_libraries(subscriber ${catkin_LIBRARIES})
add_dependencies(subscriber beginner_tutorials_generate_messages_cpp)

add_executable(pose_sub src/pose_sub.cpp)
target_link_libraries(pose_sub ${catkin_LIBRARIES})
add_dependencies(pose_sub beginner_tutorials_generate_messages_cpp)

add_executable(basic_controls_test src/basic_controls_test.cpp)
target_link_libraries(basic_controls_test
   ${catkin_LIBRARIES}
)


#####
#add_executable(selection src/selection.cpp)
#target_link_libraries(selection
#   ${catkin_LIBRARIES}
#)

#add_executable(pong src/pong.cpp)
#target_link_libraries(pong
#   ${catkin_LIBRARIES}
#)

#add_executable(cube src/cube.cpp)
#target_link_libraries(cube
#   ${catkin_LIBRARIES}
#)

#add_executable(menu src/menu.cpp)
#target_link_libraries(menu
#   ${catkin_LIBRARIES}
#)

#add_executable(point_cloud src/point_cloud.cpp)
#target_link_libraries(point_cloud
#   ${catkin_LIBRARIES}
#)
######
#############
## Install ##
#############

install(PROGRAMS
  scripts/basic_controls_test.py
  #scripts/cube.py
  #scripts/menu.py
  scripts/simple_marker_test.py
  scripts/pos_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS
  simple_marker_test
  basic_controls_test
  test_marker
  test2_marker
  publisher
  subscriber
  pose_sub
  #selection
  #pong
  #cube
  #menu
  #point_cloud
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
