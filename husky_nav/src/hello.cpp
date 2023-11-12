#include <iostream>
#include <vector>
#include <cstdlib>

// Gazebo libraries
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/GetModelState.h>

// Poses
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

// Include the ROS library
#include <ros/ros.h>

// Navigation
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

// Include pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// Include PointCloud2 message
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
 
// CV Library
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>

static const std::string CLOUD_TOPIC = "/realsense/depth/color/points";
static const std::string LASER_TOPIC = "/front/scan";
static const std::string ODOM_TOPIC = "/odometry/filtered";

// SLAM[x][y] captures x=[0.01x,0.01x+0.01),y=[0.01y,0.01y+0.01)
bool SLAM[1000][1000];

double robot_x; double robot_y; double robot_z; double robot_dir;

double rotate(double x1, double y1, double theta, double& xr, double& yr)
{
  xr = x1*cos(theta)-y1*sin(theta);
  yr = x1*sin(theta)+y1*cos(theta);
}

ros::ServiceClient client; 
  /*
  cv::Mat drawing(1200, 1200, CV_8UC3, cv::Scalar(228, 229, 247));
  cv::Rect r(100, 100, 50, 50);
  cv::rectangle(drawing, r, cv::Scalar(255, 255, 0), -1);
  for (int i=0; i<1000; i += 1) {
    for (int j=0; j<1000; j += 1) {
      if (SLAM[i][j] == 1) {
        // std::cout << "located " << i << " " << j << std::endl;
        cv::Rect rect(i, j, 1, 1);
        cv::rectangle(drawing, rect, cv::Scalar(255, 255, 0), -1);
      }
    }
  }
  
  cv::imshow("PCL DISPLAY", drawing);
  cv::waitKey(1);  
*/

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{   
    nav_msgs::MapMetaData info = msg->info;
    ROS_INFO("Got map %d %d", info.width, info.height);
    geometry_msgs::Pose origin = info.origin;
    ROS_INFO("Robot coordinates: x: %f, y: %f, z: %f \n", origin.position.x, origin.position.y, origin.position.z);
    ROS_INFO("Robot rotation coordinates: %f, %f, %f, %f \n", origin.orientation.x, origin.orientation.y, origin.orientation.z, origin.orientation.w);

}
void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

  std::vector<std::pair<double,double>> scan_map;
  std::cout << msg->angle_min << std::endl;
  double angle = msg->angle_min;
  for (int i=0; i<(int) msg->ranges.size(); i++) {
    if (!isinf(msg->ranges[i])) {
      double px = cos(angle) * msg->ranges[i];
      double py = sin(angle) * msg->ranges[i];
      double fx; double fy;
      rotate(px, py, robot_dir, fx, fy);
      // std::cout << "found at " << px << " " << py << std::endl;
      scan_map.push_back(std::pair<double, double>(fx+robot_x, fy+robot_y));
    }
    angle += msg->angle_increment;
  }

  for (int i=0; i<scan_map.size(); i++) {
    int ind_x = (scan_map[i].first)/0.01; int ind_y = (scan_map[i].second)/0.01;
    if (0 <= ind_x and ind_x < 1000 and 0 <= ind_y and ind_y < 1000 and SLAM[ind_x][ind_y] == 0) {
      // ROS_INFO("%f, %f, %f", pt_.x+robot_x, pt_.y+robot_y, pt_.z+robot_z);
      // std::cout << "found" << std::endl;
      SLAM[ind_x][ind_y] = 1;
    }
  }

  cv::Mat drawing(1200, 1200, CV_8UC3, cv::Scalar(228, 229, 247));
  for (int i=0; i<1000; i += 1) {
    for (int j=0; j<1000; j += 1) {
      if (SLAM[i][j] == 1) {
        // std::cout << "located " << i << " " << j << std::endl;
        cv::Rect rect(i, j, 3, 3);
        cv::rectangle(drawing, rect, cv::Scalar(255, 255, 0), -1);
      }
    }
  }
  cv::Rect rect(robot_x/0.01, robot_y/0.01, 15, 15);
  cv::rectangle(drawing, rect, cv::Scalar(255, 128, 0), -1);

  cv::imshow("PCL DISPLAY", drawing);
  cv::waitKey(1);  
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{

  // RO_INFO("x: %f, y: %f, z: %f", odom_husky_pose.pose.position.x, odom_husky_pose.pose.position.y, odom_husky_pose.pose.position.z); 
  
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
  tf::Quaternion q(
        husky_pose.orientation.x,
        husky_pose.orientation.y,
        husky_pose.orientation.z,
        husky_pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  robot_dir = yaw;
  std::cout << "The robot is located at: " << robot_x << " " << robot_y << " " << robot_z << " in direction" << roll << " " << pitch << " " << yaw << std::endl;
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
  ros::Subscriber odom_sub_ = nh.subscribe(ODOM_TOPIC, 1, odom_callback);
  ros::Subscriber model_states_subscriber = nh.subscribe("/gazebo/model_states", 100, model_states_callback);
  ros::Subscriber laser_sub = nh.subscribe(LASER_TOPIC, 100, laser_callback);
  ros::Subscriber map_sub = nh.subscribe("/map", 2000, map_callback);
  /*
  client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  gazebo_msgs::GetModelState get_model_state;
  get_model_state.request.model_name = "husky";
  client.call(get_model_state);
  std::cout << "The position is " << (get_model_state.response.pose).x << std::endl;
  */

  // pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  ros::spin();
  return 0;
}
