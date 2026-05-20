import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_robotics_challenge = get_package_share_directory('robotics_challenge')

    eval_node = Node(
        package="robotics_challenge",
        executable="evaluation",
        name="robotics_challenge_evaluation",
        output="screen",
        parameters=[
            {"output_file": os.path.join(pkg_robotics_challenge, "metrics.txt")},
            {"base_frame": "base_footprint"},
            {"global_frame": "map"},
            {"max_lin_vel": 0.25},
            {"max_ang_vel": 1.2},
            {"goal_tolerance": 0.15},
            {"min_collision_dist": 0.12},
            { "goals_config": os.path.join(pkg_robotics_challenge, "config", "navigation_goals.yaml")}
        ]
    )

    return LaunchDescription([
        eval_node
    ])
