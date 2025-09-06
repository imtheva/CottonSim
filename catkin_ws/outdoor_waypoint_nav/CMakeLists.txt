cmake_minimum_required(VERSION 3.0.2)
project(outdoor_waypoint_nav)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslaunch
  roslib
)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

## Find GeographicLib
find_package(GeographicLib REQUIRED)

## Declare a catkin package
catkin_package()

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

## Declare the C++ executables
add_executable(gps_waypoint src/gps_waypoint.cpp)
add_executable(gps_waypoint_cv src/gps_waypoint_cv.cpp)
add_executable(safety_node src/safety_node.cpp)

## Link libraries to the executables
target_link_libraries(gps_waypoint
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  roslib
)
target_link_libraries(gps_waypoint_cv
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  roslib
)

target_link_libraries(safety_node
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
  roslib
)

## Add dependencies to ensure build order
add_dependencies(gps_waypoint ${catkin_EXPORTED_TARGETS})
add_dependencies(gps_waypoint_cv ${catkin_EXPORTED_TARGETS})
add_dependencies(safety_node ${catkin_EXPORTED_TARGETS})

## Install targets (optional, required for installation)
install(TARGETS gps_waypoint safety_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS gps_waypoint_cv safety_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Install other files, such as launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
