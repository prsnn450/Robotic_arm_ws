from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.event_handlers import OnProcessExit
import os
import xacro

def generate_launch_description():
    # Path to Robot's Xacro File
    pkg_path = get_package_share_directory("kuka_arm_pkg")
    xacro_file = os.path.join(pkg_path, 'urdf', 'kr210.urdf.xacro')
    default_model_path = os.path.join(pkg_path, 'urdf', 'kr210.urdf.xacro')

    # Processing Xacro File
    xacro_parser = xacro.process_file(xacro_file)
    robot_description_config = xacro_parser.toxml()

    # Feeding URDF to ROS
    parameters = {'robot_description': robot_description_config}

    # Declare the robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[parameters]
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Gazebo node
    gazebo_node = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Spawn robot node
    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'kuka_arm',
        ],
        output='screen'
    )

    # Load controllers
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    load_kuka_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'kuka_arm_controller'],
        output='screen'
    )

    load_kuka_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'kuka_gripper_controller'],
        output='screen'
    )

    # Delaying the loading of controllers until Gazebo and the robot are fully spawned
    delay_joint_state_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[load_joint_state_controller]
        )
    )

    delay_kuka_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_kuka_arm_controller]
        )
    )

    delay_kuka_gripper_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=load_kuka_arm_controller,
            on_exit=[load_kuka_gripper_controller]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot urdf file'
        ),
        robot_state_publisher_node,
        gazebo_node,
        spawn_robot_node,
        delay_joint_state_controller,
        delay_kuka_arm_controller,
        delay_kuka_gripper_controller
    ])
