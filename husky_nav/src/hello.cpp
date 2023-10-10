#include <iostream>
#include <vector>

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

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  ROS_INFO("inside callback"); 
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  pcl::fromROSMsg(*cloud_msg, cloud_);
  std::vector<pcl::PointXYZ> data;
  
  for (int i=0; i<cloud_.size(); i++) {
    data.push_back(cloud_.points[i]);
    auto pt_ = data[i];
    // ROS_INFO("%f, %f, %f", pt_.x, pt_.y, pt_.z);
  }
   
  cv::Mat drawing(360, 480, CV_8UC3, cv::Scalar(228, 229, 247));
  cv::imshow("PCL DISPLAY", drawing);
  cv::waitKey(1);  
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{

  ROS_INFO("x: %f, y: %f, z: %f", odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z); 

}
/*
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
         {
             // Container for original & filtered data
             pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
             pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
             pcl::PCLPointCloud2 cloud_filtered;

             // Convert to PCL data type
             pcl_conversions::toPCL(*cloud_msg, *cloud);

             // Perform the actual filtering
             pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
             sor.setInputCloud (cloudPtr);
             sor.setLeafSize (0.1, 0.1, 0.1);
             sor.filter (cloud_filtered);

             // Convert to ROS data type
             sensor_msgs::PointCloud2 output;
             pcl_conversions::moveFromPCL(cloud_filtered, output);

             // Publish the data
             pub.publish (output);
         }
*/

int main(int argc, char** argv) 
{
  ros::init (argc, argv, "cloud_sub_pub");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  ros::Subscriber cloud_sub_ = nh.subscribe(CLOUD_TOPIC, 1, cloud_callback);
  ros::Subscriber odom_sub_ = nh.subscribe(ODOM_TOPIC, 1, odom_callback);
  // pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  ros::spin();
}
