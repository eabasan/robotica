import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_localization = get_package_share_directory('localization')
    pkg_path_planner = get_package_share_directory('path_planner')

    declare_rviz_arg = DeclareLaunchArgument(
        'rviz_conf',
        default_value='planning.rviz',
        description='RViz configuration file'
    )
    rviz_path = os.path.join(pkg_path_planner, 'config', 'planning.rviz') 
    # rviz_path = os.path.join(pkg_path_planner, 'config', LaunchConfiguration('rviz_conf'))

    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_localization, 'launch', 'amcl.launch.py')
        ),
        launch_arguments={
            'initial_pose_x': '0.0',
            'initial_pose_y': '0.0',
            'initial_pose_a': '0.0',
            'rviz_conf': rviz_path
        }.items()
    )

    global_costmap_params = os.path.join(pkg_path_planner, 'config', 'global_costmap_params.yaml')

    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='nav2_costmap_2d',
        output='screen',
        parameters=[global_costmap_params, 
            {'use_sim_time': True}],
        remappings=[('map', 'map')]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'bond_timeout': 0.0},
            {'node_names': ['nav2_costmap_2d']}
        ]
    )

    path_planner_node = Node(
        package='path_planner',
        executable='path_planner',
        name='path_planner',
        output='screen',
        parameters=[{
            'base_frame_id': 'base_footprint',
            'global_frame_id': 'map',
            'update_map': False,
            'use_sim_time': True   # HE AÑADIDO ESTO
        }],
        remappings=[
            ('goal', 'goal_pose'),
            ('costmap_2d/costmap/costmap', 'costmap')
        ]
    )

    return LaunchDescription([
        declare_rviz_arg,
        amcl_launch,
        costmap_node,
        lifecycle_manager,
        path_planner_node
    ])
