cmake_minimum_required(VERSION 2.8.3)
project(rrbot_description)

#find_package(catkin REQUIRED)
find_package(catkin REQUIRED interactive_markers roscpp visualization_msgs tf)

#catkin_package()
catkin_package(
CATKIN_DEPENDS interactive_markers roscpp visualization_msgs tf
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
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS
  simple_marker_test
  basic_controls_test
  test_marker
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



