import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='turtlebot3_dqn_stage4.world',
        description='Name of the Gazebo world'
    )

    rviz_conf_arg = DeclareLaunchArgument(
        'rviz_conf',
        default_value=os.path.join(get_package_share_directory('localization'), 'config', 'map_conf.rviz'),
        description='RViz config file'
    )

    # Build the full path to the world file using PathJoinSubstitution
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    world_path = PathJoinSubstitution([turtlebot3_gazebo_dir, 'worlds', LaunchConfiguration('world_name')])

    # Include turtlebot3_sim launch
    turtlebot_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('localization'), 'launch', 'turtlebot3_sim.launch.py')
        ),
        launch_arguments={
            'world_name_and_path': world_path,
            'x_pos': '0.0',
            'y_pos': '0.0',
            'z_pos': '0.1',
            'yaw_pos': '0.0'
        }.items()
    )

    # SLAM Toolbox launch
    #ros2 launch slam_toolbox online_async_launch.py
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        )
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_conf')],
        name='rviz2'
    )

    return LaunchDescription([
        world_name_arg,
        rviz_conf_arg,
        turtlebot_sim_launch,
        slam_toolbox_launch,
        rviz_node
    ])