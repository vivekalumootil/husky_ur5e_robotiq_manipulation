#include <iostream>
#include <vector>
#include <utility>
#include <cstdlib>

// TF2
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// Poses
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/PointStamped.h"

// Include the ROS library
#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "moveit_arm_control");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Initialize MoveIt
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    // Set a target joint configuration
    std::vector<double> joint_values = {-2.41, -2.16, -0.38, -0.58, 1.50, 3.14}; // Example joint angles

    // Set the joint target
    move_group.setJointValueTarget(joint_values);

    // Plan and execute the trajectory
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (success) {
        ROS_INFO("Planning successful. Executing the trajectory...");
        move_group.execute(my_plan);
    } else {
        ROS_ERROR("Planning failed.");
    }

    std::vector<double> current_joint_values = move_group.getCurrentJointValues();

    // Print the current joint values
    ROS_INFO("Current joint values:");
    for (size_t i = 0; i < current_joint_values.size(); ++i) {
        ROS_INFO_STREAM("Joint " << i << ": " << current_joint_values[i]);
    }

    ros::shutdown();
    return 0;
}
