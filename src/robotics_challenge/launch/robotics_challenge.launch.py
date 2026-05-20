import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_path_planner = get_package_share_directory('path_planner')
    pkg_robotics_challenge = get_package_share_directory('robotics_challenge')
    rviz_path = os.path.join(pkg_robotics_challenge, 'config', 'robotics_challenge.rviz')

    path_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path_planner, 'launch', 'path_planner.launch.py')
        ),
        launch_arguments={
            'rviz_conf': rviz_path
        }.items()
    )

    # Launch the robot controller node
    robot_controller_node = Node(
        package="robot_controller",
        executable="robot_controller",
        name="robot_controller",
        output="screen",
        parameters=[
            #{"use_sim_time": True},  # he añadido esto yo
            {"robot_vel_topic": "cmd_vel"},
            {"robot_scan_topic": "scan"},
            {"max_lin_vel": 0.25},
            {"max_ang_vel": 1.2},
            {"control_rate": 10},
        ],
    )

    return LaunchDescription([
        robot_controller_node,
        path_planner_launch
    ])
