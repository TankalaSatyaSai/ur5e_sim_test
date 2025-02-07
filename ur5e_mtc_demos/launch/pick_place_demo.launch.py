from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    package_name_moveit_config = 'ur5e_moveit_config'
    robot_name_str = 'ur5e'

    # Get package path
    pkg_share_moveit_config = get_package_share_directory(package_name_moveit_config)

    # Construct file paths using robot name string
    config_path = os.path.join(pkg_share_moveit_config, 'config', robot_name_str)

    # Define all config file paths
    initial_positions_file_path = os.path.join(config_path, 'initial_positions.yaml')
    joint_limits_file_path = os.path.join(config_path, 'joint_limits.yaml')
    kinematics_file_path = os.path.join(config_path, 'kinematics.yaml')
    moveit_controllers_file_path = os.path.join(config_path, 'moveit_controllers.yaml')
    srdf_model_path = os.path.join(config_path, f'{robot_name_str}.srdf')
    pilz_cartesian_limits_file_path = os.path.join(config_path, 'pilz_cartesian_limits.yaml')

    # Create MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder(robot_name_str, package_name=package_name_moveit_config)
        .trajectory_execution(file_path=moveit_controllers_file_path)
        .robot_description_semantic(file_path=srdf_model_path)
        .joint_limits(file_path=joint_limits_file_path)
        .robot_description_kinematics(file_path=kinematics_file_path)
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
            default_planning_pipeline="ompl"
        )
        .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path)
        .to_moveit_configs()
    )

    planning_scene_monitor_parameters = {
        "publish_robot_description": False,
        "publish_robot_description_semantic": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "planning_scene_monitor_options": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "attached_collision_object_topic": "/move_group/planning_scene_monitor",
            "publish_planning_scene_topic": "/move_group/publish_planning_scene",
            "monitored_planning_scene_topic": "/move_group/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        },
    }

    # MoveIt capabilities
    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

    # MTC Demo node
    pick_place_demo = Node(
        package="ur5e_mtc_demos",
        executable="mtc_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True},
            {'start_state': {'content': initial_positions_file_path}},
            move_group_capabilities,
            # trajectory_execution,
            planning_scene_monitor_parameters
        ],
    )

    return LaunchDescription([pick_place_demo])