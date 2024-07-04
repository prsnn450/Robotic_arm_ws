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
- **Launch Gazebo and Rviz with the moveit configuration for the Robotic Arm:**
  ```sh
  ros2 launch kuka_arm_pkg 3_moveit_kuka_arm.launch.py
  ```

