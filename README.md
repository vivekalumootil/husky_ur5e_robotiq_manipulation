# husky_ur5e_robotiq_manipulation

This repository is meant to facilitate setting up a Husky robot with a UR5e arm and a Robotiq gripper. ROS Noetic is the only ROS version currently supported. You need a proper Catkin workspace for this tutorial. 

## Installing this repository
Go to the /src directory of your Catkin workspace. 
Run this to clone the repository:
```sh
git clone https://github.com/vivekalumootil/husky_ur5e_robotiq_manipulation.git
```
## Husky simulator dependency
To install the Husky simulator packages:
```sh
sudo apt-get install ros-noetic-husky-simulator
```
## Manipulators
There are two possible options this repository provides. You can have the husky with a UR5 arm manipulator, or with a UR5e arm and a Robotiq gripper.

## Starting up the gazebo world
Spawn a new terminal window and move to the root directory of your workspace.
```sh
1. catkin build
2. source devel/setup.bash
```

### Only UR5 Arm
To attach the arm to your Husky robot and start the simulation:
```sh
1. export HUSKY_URDF_EXTRAS=$(catkin_find husky_ur_description urdf/husky_ur5_e_description.urdf.xacro --first-only)
2. roslaunch ucla_husky_bringup simulation.launch
```

### UR5 Arm and Robotiq Gripper
To attach the arm and gripper to your Husky robot and start the simulation:
```sh
1. export HUSKY_URDF_EXTRAS=$(catkin_find husky_ur_description urdf/husky_ur5_e_gripper_description.urdf.xacro --first-only)
2. roslaunch ucla_husky_bringup_gripper simulation.launch
```
## Launching the MoveIt package
Spawn a new terminal window and move to the root directory of your workspace. 
```sh
source devel/setup.bash
```
### Only UR5 Arm
```sh
roslaunch husky_ur_moveit_config husky_ur_moveit_planning_execution.launch
```
### UR5 Arm and Robotiq Gripper
```sh
roslaunch husky_ur_robotiq_2f_85_moveit_config husky_ur_robotiq_2f_85_moveit_planning_execution.launch
```
## Rviz controller
Spawn a new terminal window. 
```sh
1. source devel/setup.bash
2. rviz
```
In the displays menu, change the *Fixed Frame* to be *odom*. Then click the *Add* button --> *MotionPlanning* --> *OK*. 
Select the planning group which corresponds to the manipulator you would like to control. *manipulator* is for the arm and *gripper* is for the gripper.
Then drag the colored arrows around or adjust the sliders in the joint tab and press *Plan & Execute*. 

