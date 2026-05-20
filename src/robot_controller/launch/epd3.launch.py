from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='epd3.sdf',
        description='Name of the Gazebo world'
    )

    package_share = get_package_share_directory('robot_controller')

    # Get package paths
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    worlds_dir = os.path.join(turtlebot3_gazebo_dir, 'worlds')
    local_worlds_dir = os.path.join(get_package_share_directory('robot_controller'), 'worlds')
    
    # Build the full path to the world file using PathJoinSubstitution
    world_path = PathJoinSubstitution([local_worlds_dir, LaunchConfiguration('world_name')])
    
    # Include the turtlebot3 simulation launch
    turtlebot3_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                package_share,
                'launch',
                'turtlebot3_sim.launch.py'
            ])
        ]),
        #launch_arguments=[
        #    ('world_name_and_path', local_world_arg),
        #    ('x_pos', '0.0'),
        #    ('y_pos', '0.0'),
        #    ('z_pos', '0.1'),
        #    ('yaw_pos', '0.0'),
        #]
        launch_arguments={
            'world_name_and_path': world_path,
            'x_pos': '0.0',
            'y_pos': '0.0',
            'z_pos': '0.1',
            'yaw_pos': '0.0'
        }.items()
    )
    
    # Include the path publisher launch
    path_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                package_share,
                'launch',
                'path_publisher.launch.py'
            ])
        ]),
        launch_arguments={
            'path_file_name': 'path.yaml'
        }.items()
    )
    
    # Launch the robot controller node
    robot_controller_node = Node(
        package='robot_controller',
        executable='robot_controller',
        name='robot_controller',
        output='screen',
        parameters=[
            {'robot_vel_topic': 'cmd_vel'},
            {'robot_scan_topic': 'scan'},
            {'max_lin_vel': 0.3},
            {'max_ang_vel': 1.2},
            {'control_rate': 10},
        ]
    )

    # Timer: wait 6 seconds before launching the node
    delayed_node_action = TimerAction(
        period=6.0,
        actions=[robot_controller_node]
    )
    
    return LaunchDescription([
        world_name_arg,
        turtlebot3_sim_launch,
        path_publisher_launch,
        delayed_node_action
    ])
