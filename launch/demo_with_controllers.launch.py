import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Create MoveIt configs
    moveit_configs = MoveItConfigsBuilder("test", package_name="test_moveit_config")\
        .robot_description(file_path="config/test.urdf.xacro")\
        .robot_description_semantic(file_path="config/test.srdf")\
        .trajectory_execution(file_path="config/moveit_controllers.yaml")\
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"],
        )\
        .to_moveit_configs()
    """
    ros2_controller_path = os.path.join(
        get_package_share_directory("test_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_configs.robot_description, ros2_controller_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    """
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_configs.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )
    
    rviz_config_path = os.path.join(
        get_package_share_directory("test_moveit_config"),
        "config",
        "moveit.rviz",
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],  # Fixed the equal sign
        parameters=[
            moveit_configs.robot_description,
            moveit_configs.robot_description_semantic,
            moveit_configs.planning_pipelines,
            moveit_configs.robot_description_kinematics,
        ],
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_configs.robot_description],
        output="screen",
    )

    return LaunchDescription([

        robot_state_publisher_node,
        move_group_node,
        rviz_node,
    ])