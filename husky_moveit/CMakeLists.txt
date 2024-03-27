cmake_minimum_required(VERSION 3.0.2)
project(husky_moveit)

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
  moveit_ros_planning
  moveit_ros_planning_interface
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  LIBRARIES navigate
  CATKIN_DEPENDS rospy roscpp std_msgs pcl_conversions pcl_ros cv_bridge image_transport nav_msgs gazebo_msgs geometry_msgs tf tf2_ros tf2_geometry_msgs moveit_ros_planning moveit_ros_planning_interface
)

add_executable(ur5 src/node.cpp)
target_link_libraries(ur5 ${catkin_LIBRARIES})
