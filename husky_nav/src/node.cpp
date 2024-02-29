#include <iostream>
#include <vector>
#include <utility>
#include <cstdlib>

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

typedef std::vector<std::pair<double, double>> vPT;
typedef std::vector<std::tuple<double, double, double>> cPT;
typedef std::pair<double, double> PT;
typedef std::tuple<double, double, double> CT;

#define PI 3.1415
#define RAD2DEG 57.2957

static const std::string CLOUD_TOPIC = "/realsense/depth/color/points";
static const std::string ODOM_TOPIC = "/odometry/filtered"; 
// ros::Publisher pub;
double global_x; double global_y; double global_z;
CT my_center;

double dist_2d(double x1, double y1, double x2, double y2)
{
      return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

void find_circle(double x1, double y1, double x2, double y2, double x3, double y3, double& cx, double& cy, double& rad)
{
      double J = (x1-x2)*(y1-y3) - (x1-x3)*(y1-y2);
      double U = x1*x1+y1*y1-x3*x3-y3*y3;
      double T = x1*x1+y1*y1-x2*x2-y2*y2;
      cx = -(y1-y2)*U + (y1-y3)*T;
      cx = cx/(2*J);
      cy = (x1-x2)*U - (x1-x3)*T;
      cy = cy/(2*J);
      rad = dist_2d(x2, y2, cx, cy);
}

static double get_angle(double dx, double dy)
{
       if (dx > 0.1) {
         return atan(dy/dx);
       }
       else if (dx < 0.1) {
         return atan(dy/dx) + PI;
       }
       else {
       	 return PI/2;
       }
}

static bool angle_comparison(const PT &a, const PT &b)
{
      return get_angle(a.first, a.second) < get_angle(b.first, b.second);
}

CT find_cylinders(vPT points)
{
    
      cPT centers;
      int nsize = points.size();
      std::cout << "size: " << points.size() << std::endl; 
      // sort by angle
      std::sort(points.begin(), points.end(), angle_comparison);
      
      // Detect centers
      int t = points.size();
      int sample = 1000;
      double bx = 0; double by = 0; double br = -1; double bs = 100000; 
      for (int i=0; i<t-7; i++) {
	int a = i;
	int b = i+3;
	int c = i+6;
	if (a == b or b == c or a == c)
		break;
        double dx; double dy; double r;
	std::vector<double> dists;
        find_circle(points[a].first, points[a].second, points[b].first, points[b].second, points[c].first, points[c].second, dx, dy, r);
        int score = 0;
        for (int j=0; j<points.size(); j++) {
            if (abs(dist_2d(points[j].first, points[j].second, dx, dy)-r) <= 0.2) {
              	dists.push_back(abs(dist_2d(points[j].first, points[j].second, dx, dy)-r));
		score += 1;
            }
        }
	std::sort(dists.begin(), dists.end());
	if (score > 10 and r <= 0.55 and r >= 0.45) {
	    int metric = 1;
	    for (int ind=0; ind<10; ind++) {
	       metric += dists[i];
	    }
	    metric /= 10;
	    metric /= score;
	    metric * abs(r-0.5);
	    if (metric < bs) {
	      bx = dx; by = dy; br = r; bs = metric;
	    }
	    // std::cout << points[i].first << " " << points[i].second << " " << points[i+1].first << " " << points[i+1].second << " " << points[i+2].first << " " << points[i+2].second << std::endl;
	    std::cout << bx << " " << by << " " << br << std::endl;
	}
      }
      std::cout << "lowest metric found: " << bs << std::endl;
      return CT(bx, by, br);
}

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  pcl::fromROSMsg(*cloud_msg, cloud_);
  std::vector<pcl::PointXYZ> data;

geometry_msgs::PoseStamped init_;
  geometry_msgs::PoseStamped final_;
  init_.header  = cloud_msg->header;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_listener(buffer_);
  geometry_msgs::TransformStamped depth_to_map; 

  depth_to_map = buffer_.lookupTransform("odom", "front_realsense_gazebo", ros::Time(0), ros::Duration(1.0));

  vPT mmap;
  for (int i=0; i<cloud_.size(); i++) {
    // data.push_back(cloud_.points[i]);
    auto pt_ = cloud_.points[i];
    init_.pose.position.x = pt_.x;
    init_.pose.position.y = pt_.y;
    init_.pose.position.z = pt_.z;
    
    tf2::doTransform(init_, final_, depth_to_map); // robot_pose is the PoseStamped I want to transform
    // ROS_INFO("realsense: %f, %f, %f", pt_.x, pt_.y, pt_.z);
    double dx = final_.pose.position.x;
    double dy = final_.pose.position.y;
    double dz = final_.pose.position.z;
    if (dz >= 0.5 and dz <= 0.501) {
      // ROS_INFO("map: %f, %f, %f", dx, dy, dz);
      // auto dp_ = pcl::PointXYZ(dx, dy, dz);
      // data.push_back(dp_);
      mmap.push_back(PT(dx, dy));
    }
    if (dz >= 0.1 and dz <= 0.9) {
      // ROS_INFO("map: %f, %f, %f", dx, dy, dz);
      auto dp_ = pcl::PointXYZ(dx, dy, dz);
      data.push_back(dp_);
    }
  }

  cv::Mat drawing(900, 900, CV_8UC3, cv::Scalar(228, 229, 247));
  my_center = find_cylinders(mmap);
  double px = std::get<0>(my_center);
  double py = std::get<1>(my_center);
  double R = std::get<2>(my_center);
  if (R > 0) {
	std::cout << "Found circle with center (" << px << ", " << py << ") with radius " << R << std::endl;
        cv::circle(drawing, cv::Point(100*px+300, 100*py+300), 100*R, cv::Scalar(214, 140, 43), -1);
  }
  for (int i=0; i<mmap.size(); i++) {
       // std::cout << data[i].x << " " << data[i].y << " " << data[i].z << std::endl;
       cv::Rect rect(100*mmap[i].first+300, 100*mmap[i].second+300, 2, 2);
       cv::rectangle(drawing, rect, cv::Scalar(255, 255, 0), -1);
  }

  cv::imshow("CROSS-SECTION DISPLAY", drawing);
  cv::waitKey(1);

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
  srand((unsigned) time(NULL));
  ros::init (argc, argv, "cloud_sub_pub");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  ros::Subscriber cloud_sub_ = nh.subscribe(CLOUD_TOPIC, 1, cloud_callback);
  ros::Subscriber model_states_subscriber = nh.subscribe("/gazebo/model_states", 100, model_states_callback);
  ros::Subscriber odom_sub_ = nh.subscribe(ODOM_TOPIC, 1, odom_callback);
  ros::spin();
}
