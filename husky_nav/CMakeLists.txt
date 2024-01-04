cmake_minimum_required(VERSION 3.0.2)
project(husky_nav)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  pcl_conversions
  pcl_ros
  cv_bridge
  image_transport
  nav_msgs
  gazebo_msgs
  geometry_msgs
  tf
  tf2_ros
  tf2_geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  LIBRARIES navigate
  CATKIN_DEPENDS rospy roscpp std_msgs pcl_conversions pcl_ros cv_bridge image_transport nav_msgs gazebo_msgs geometry_msgs tf tf2_ros tf2_geometry_msgs
)

add_executable(navigate src/node.cpp)
target_link_libraries(navigate ${catkin_LIBRARIES})
