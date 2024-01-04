#include <iostream>
#include <vector>

// Gazebo libraries
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/GetModelState.h>

// TF2
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// Poses
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/PointStamped.h"

// Include the ROS library
#include <ros/ros.h>

// Navigation
#include <nav_msgs/Odometry.h>

// Include pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// Include PointCloud2 message
#include <sensor_msgs/PointCloud2.h>
 
// CV Library
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>

static const std::string CLOUD_TOPIC = "/realsense/depth/color/points";
static const std::string ODOM_TOPIC = "/odometry/filtered"; 
// ros::Publisher pub;
double global_x; double global_y; double global_z;

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  ROS_INFO("ENTERING CALLBACK"); 
  ROS_INFO("_________________"); 
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  pcl::fromROSMsg(*cloud_msg, cloud_);
  std::vector<pcl::PointXYZ> data;

  geometry_msgs::PoseStamped init_;
  geometry_msgs::PoseStamped final_;
  init_.header  = cloud_msg->header;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_listener(buffer_);
  geometry_msgs::TransformStamped depth_to_map; 

  depth_to_map = buffer_.lookupTransform("map", "front_realsense_gazebo", ros::Time(0), ros::Duration(1.0));

  for (int i=0; i<cloud_.size(); i++) {
    data.push_back(cloud_.points[i]);
    auto pt_ = data[i];
    init_.pose.position.x = pt_.x;
    init_.pose.position.y = pt_.y;
    init_.pose.position.z = pt_.z;
    
    tf2::doTransform(init_, final_, depth_to_map); // robot_pose is the PoseStamped I want to transform
    // ROS_INFO("realsense: %f, %f, %f", pt_.x, pt_.y, pt_.z);
    double dx = final_.pose.position.x;
    double dy = final_.pose.position.y;
    double dz = final_.pose.position.z;
    if ((abs(pt_.x) > 1 or abs(pt_.y) > 1 or abs(pt_.z) > 1) and dz >= 0) {
      // ROS_INFO("map: %f, %f, %f", dx, dy, dz);
    }
  }

}

void model_states_callback(gazebo_msgs::ModelStates model_states) {
  int ind = 0;
  for (ind=0; ind<(model_states.name).size(); ind++) {
    if (model_states.name[ind] == "husky") {
      break;
    }
  }
  geometry_msgs::Pose husky_pose = model_states.pose[ind];
  global_x = ((husky_pose).position).x;
  global_y = ((husky_pose).position).y;
  global_z = ((husky_pose).position).z;
  // std::cout << "The robot is located at: " << global_x << " " << global_y << " " << global_z << std::endl;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  //ROS_INFO("x: %f, y: %f, z: %f", odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z); 
}

int main(int argc, char** argv) 
{
  ros::init (argc, argv, "cloud_sub_pub");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  ros::Subscriber cloud_sub_ = nh.subscribe(CLOUD_TOPIC, 1, cloud_callback);
  ros::Subscriber model_states_subscriber = nh.subscribe("/gazebo/model_states", 100, model_states_callback);
  ros::Subscriber odom_sub_ = nh.subscribe(ODOM_TOPIC, 1, odom_callback);
  ros::spin();
}
