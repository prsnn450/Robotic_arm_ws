# Robotic_arm_ws
## Overview
This repository contains the source code and configuration files for the robotic arm controlled using ROS2 and MoveIt. The project simulates the robotic arm's movements using Gazebo and visualizes them using Rviz.

## Getting Started

These instructions will help you set up the project on your local machine.

### Install Dependencies

Install the necessary dependencies using the following commands:

```sh
sudo apt-get install ros-humble-gazebo-ros
sudo apt-get install ros-humble-gazebo-ros2-control
sudo apt-get install ros-humble-control*
sudo apt-get install ros-humble-gazebo-plugins
sudo apt-get install ros-humble-moveit-visual-tools
sudo apt-get install ros-humble-graph-msgs
```

## Running Project

### Clone the Repository
To clone this repository to your local machine, use the following command:
```sh
git clone https://github.com/prsnn450/Robotic_arm_ws.git
```

### Building the workspace
Navigate to that workspace and run commands
```sh
colcon build
source install/setup.bash
```

### Run the Launch Files

Use the following launch files to execute different components of the robotic arm project:

- **Launch Gazebo with ros2_contol for the Robotic Arm:**
  ```sh
  ros2 launch kuka_arm_pkg 2_gazebo_kuka_arm.launch.py
  ```
  Open a new termainal and run the below command for the joint trajectory control
  ```sh
  ros2 run kuka_arm_pkg jtc_node
  ```
  Or you can control the joint trajectory using ros2 topic, <br>
  To control the arm_controller:
  ```sh
  ros2 topic pub --once /kuka_arm_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
  'joint_names': ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'],
  'points': [{
    'positions': [0.0, -0.5, 0.5, -1.0, 1.0, 0.0],
    'time_from_start': { 'sec': 2, 'nanosec': 0 }
  }, {
    'positions': [-0.5, 0.0, 0.5, 0.0, -1.0, 0.5],
    'time_from_start': { 'sec': 4, 'nanosec': 0 }
  }]
  }"
  ```
  To control the gripper_controller:
  ```sh
  ros2 topic pub --once /kuka_gripper_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
    'joint_names': ['left_gripper_finger_joint', 'right_gripper_finger_joint'],
    'points': [{
      'positions': [0.04, 0.04],
      'time_from_start': { 'sec': 1, 'nanosec': 0 }
    }, {
      'positions': [0.01, 0.01],
      'time_from_start': { 'sec': 2, 'nanosec': 0 }
    }]
  }"
  ```
- **Launch Gazebo and Rviz with the Moveit configuration for the Robotic Arm:**
  ```sh
  ros2 launch kuka_arm_pkg 3_moveit_kuka_arm.launch.py
  ```
- **Launch a node to control the target pose of the Robotic Arm using Moveit configuration:**
  Before running this launch file ensure 3_moveit_kuka_arm.launch.py is running on another terminal.
  Run these export command in the terminal to avoid "File Not Found Error" (change it according to your file path). 
  ```sh
  export LD_LIBRARY_PATH=/home/vboxuser/moveit_ws/install/moveit_ros_planning_interface/lib:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=/home/vboxuser/moveit_ws/install/moveit_ros_warehouse/lib:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=/home/vboxuser/moveit_ws/install/moveit_ros_planning/lib:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=/home/vboxuser/moveit_ws/install/moveit_core/lib:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=/home/vboxuser/moveit_ws/install/moveit_ros_occupancy_map_monitor/lib:$LD_LIBRARY_PATH
  export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH
  ```
  Now run the launch file using the below command:
  ```sh
  ros2 launch kuka_arm_pkg 4_moveit_node_controlling.launch.py
  ```


