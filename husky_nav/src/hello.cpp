#include <iostream>
#include <vector>

// Gazebo libraries
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/GetModelState.h>

// Poses
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

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

// SLAM[x][y] captures x=[0.01x,0.01x+0.01),y=[0.01y,0.01y+0.01)
bool SLAM[1000][1000];

double robot_x; double robot_y; double robot_z;

// ros::Publisher pub;

ros::ServiceClient client; 
void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // ROS_INFO("inside callback"); 
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  pcl::fromROSMsg(*cloud_msg, cloud_);
  std::vector<pcl::PointXYZ> data;
  
  for (int i=0; i<cloud_.size(); i++) {
    data.push_back(cloud_.points[i]);
    auto pt_ = data[i];
    // ROS_INFO("%f, %f, %f", pt_.x, pt_.y, pt_.z);
    int ind_x = pt_.x/0.01; int ind_y = pt_.y/0.01;
    if (0 <= ind_x and ind_x < 1000 and 0 <= ind_y and ind_y < 1000 and SLAM[ind_x][ind_y] == 0) {
      std::cout << "found" << std::endl;
      SLAM[ind_x][ind_y] = 1;
    }
  }

  cv::Mat drawing(360, 480, CV_8UC3, cv::Scalar(228, 229, 247));
  for (int i=0; i<1000; i++) {
    for (int j=0; j<1000; j++) {
      if (SLAM[i][j]) {
        cv::Rect rect(0.1*i, 0.1*j, 0.1, 0.1);
        cv::rectangle(drawing, rect, cv::Scalar(0, 255, 0));
      }
    }
  }
  
  cv::imshow("PCL DISPLAY", drawing);
  cv::waitKey(1);  
}

/*
void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{

  ROS_INFO("x: %f, y: %f, z: %f", odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z); 

}
*/


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

void model_states_callback(gazebo_msgs::ModelStates model_states) {
  int ind = 0;
  for (ind=0; ind<(model_states.name).size(); ind++) {
    if (model_states.name[ind] == "husky") {
      break;
    }
  }
  geometry_msgs::Pose husky_pose = model_states.pose[ind];
  robot_x = ((husky_pose).position).x;
  robot_y = ((husky_pose).position).y;
  robot_z = ((husky_pose).position).z;
  // std::cout << "The robot is located at: " << robot_x << " " << robot_y << " " << robot_z << std::endl;
}


void setup_SLAM() {
  for (int i=0; i<1000; i++) {
    for (int j=0; j<1000; j++) {
      SLAM[i][j] = 0;
    }
  }
}
/*
void get_model_state() {
  gazebo_msgs::GetModelState get_model_state;
  get_model_state.request.model_name = "husky";
  client.call(get_model_state);
  robot_x = ((get_model_state.response.pose).position).x;
  robot_y = ((get_model_state.response.pose).position).y;
  robot_z = ((get_model_state.response.pose).position).z;
}
*/

int main(int argc, char** argv) 
{
  setup_SLAM(); 
  ros::init (argc, argv, "cloud_sub_pub");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  ros::Subscriber cloud_sub_ = nh.subscribe(CLOUD_TOPIC, 1, cloud_callback);
  // ros::Subscriber odom_sub_ = nh.subscribe(ODOM_TOPIC, 1, odom_callback);
  ros::Subscriber model_states_subscriber = nh.subscribe("/gazebo/model_states", 100, model_states_callback);

  /*
  client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  gazebo_msgs::GetModelState get_model_state;
  get_model_state.request.model_name = "husky";
  client.call(get_model_state);
  std::cout << "The position is " << (get_model_state.response.pose).x << std::endl;
  */

  // pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  ros::spin();
}
