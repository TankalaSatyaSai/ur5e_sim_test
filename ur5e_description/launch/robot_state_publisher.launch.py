#!/usr/bin/env python3
#
# author: Tankala Satya Sai
# Date: November 10, 2024
# Description: Display the robotic arm with RViz
#
# This file launches the robot state publisher, joint state publisher,
# and RViz2 for visualizing the ur5e robot.

import os 
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def process_ros2_controllers_config(context):
    """Process the ROS 2 controller configuration yaml file before loading the URDF.
 
    This function reads a template configuration file, replaces placeholder values
    with actual configuration, and writes the processed file to both source and
    install directories.
 
    Args:
        context: Launch context containing configuration values
 
    Returns:
        list: Empty list as required by OpaqueFunction
    """
 
    # Get the configuration values
    tf_prefix = LaunchConfiguration('tf_prefix').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
 
    home = '/root/ws_moveit/'
 
    # Define both source and install paths
    src_config_path = os.path.join(
        home,
        'src/ur5e/ur5e_moveit_config/config',
        robot_name
    )
    install_config_path = os.path.join(
        home,
        'install/ur5e_moveit_config/share/ur5e_moveit_config/config',
        robot_name
    )
 
    # Read from source template
    template_path = os.path.join(src_config_path, 'ros2_controllers_template.yaml')
    with open(template_path, 'r', encoding='utf-8') as file:
        template_content = file.read()
 
    # Create processed content (leaving template untouched)
    processed_content = template_content.replace('${tf_prefix}', tf_prefix)
     
    # Write processed content to both source and install directories
    for config_path in [src_config_path, install_config_path]:
        os.makedirs(config_path, exist_ok=True)
        output_path = os.path.join(config_path, 'ros2_controllers.yaml')
        with open(output_path, 'w', encoding='utf-8') as file:
            file.write(processed_content)
 
    return []
 
# Define the arguments for the XACRO file
ARGUMENTS = [
    DeclareLaunchArgument('robot_name', default_value='ur5e',
                          description='Name of the robot'),
    DeclareLaunchArgument('tf_prefix', default_value='ur5e_',
                          description='Prefix for robot joints and links'),
    DeclareLaunchArgument('add_world', default_value='true',
                          choices=['true', 'false'],
                          description='Whether to add world link'),
    DeclareLaunchArgument('sim_gazebo', default_value='false',
                          choices=['true', 'false'],
                          description='Whether to use Gazebo simulation')
]
 
 
def generate_launch_description():
    # Define filenames
    urdf_package = 'ur5e_description'
    urdf_filename = 'ur.urdf.xacro'
    rviz_config_filename = 'view_robot.rviz'
 
    # Set paths to important files
    pkg_share_description = FindPackageShare(urdf_package)
    default_urdf_model_path = PathJoinSubstitution(
        [pkg_share_description, 'urdf', urdf_filename])
    default_rviz_config_path = PathJoinSubstitution(
        [pkg_share_description, 'rviz', rviz_config_filename])
 
    # Launch configuration variables
    jsp_gui = LaunchConfiguration('jsp_gui')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    urdf_model = LaunchConfiguration('urdf_model')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
 
    # Declare the launch arguments
    declare_jsp_gui_cmd = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui')
 
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')
 
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot urdf file')
 
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RVIZ')
 
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
 
    robot_description_content = ParameterValue(Command([
        'xacro', ' ', urdf_model, ' ',
        'tf_prefix:=', LaunchConfiguration('tf_prefix'), ' ',
        'add_world:=', LaunchConfiguration('add_world'), ' ',
        'sim_gazebo:=', LaunchConfiguration('sim_gazebo'), ' '
    ]), value_type=str)
 
    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content}])
 
    # Publish the joint state values for the non-fixed joints in the URDF file.
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(jsp_gui))
 
    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    start_joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(jsp_gui))
 
    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}])
 
    # Create the launch description and populate
    ld = LaunchDescription(ARGUMENTS)
    
    # Process the controller configuration before starting nodes
    ld.add_action(OpaqueFunction(function=process_ros2_controllers_config))
 
 
    # Declare the launch options
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
 
    # Add any actions
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_gui_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)
 
    return ld