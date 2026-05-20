from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, AppendEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro
import math


def generate_launch_description():
    # Declare arguments
    world_name_arg = DeclareLaunchArgument(
        'world_name_and_path',
        default_value='turtlebot3_world',
        description='Name of the Gazebo world'
    )
    
    x_pos_arg = DeclareLaunchArgument(
        'x_pos',
        default_value='0.0',
        description='X position of the robot'
    )
    
    y_pos_arg = DeclareLaunchArgument(
        'y_pos',
        default_value='0.0',
        description='Y position of the robot'
    )
    
    z_pos_arg = DeclareLaunchArgument(
        'z_pos',
        default_value='0.05',
        description='Z position of the robot'
    )
    
    yaw_pos_arg = DeclareLaunchArgument(
        'yaw_pos',
        default_value='0.0',
        description='Yaw orientation of the robot in radians'
    )
    
    # Set environment variable for TurtleBot3 model
    set_env = SetEnvironmentVariable(
        name='TURTLEBOT3_MODEL',
        value='burger'
    )

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    robot_controller_launch_dir = os.path.join( get_package_share_directory('robot_controller'), 'launch')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')


    world = LaunchConfiguration('world_name_and_path')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Get configuration values
    x_pos = LaunchConfiguration('x_pos')
    y_pos = LaunchConfiguration('y_pos')
    yaw_pos = LaunchConfiguration('yaw_pos')
    
    
    spawn_turtlebot_cmd = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
                ),
                launch_arguments={
                    'x_pose': x_pos,
                    'y_pose': y_pos
                    #'yaw_pose': yaw_pos
                }.items()
            )
        ]
    )

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'models'))


    # RViz configuration
    rviz_share = get_package_share_directory('robot_controller')
    rviz_config_path = PathJoinSubstitution([
                rviz_share,
                'config',
                'epd3.rviz'
            ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        name='rviz2'
    )
    
    # Fix TF hierarchy and positioning
    # Publish world -> odom with translation (-2,0,0) so:
    # - world is at (0,0) - center of environment
    # - odom is at (-2,0) relative to world - spawn position
    world_to_odom_transform = TimerAction(
        period=2.5,
        actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=[
                    x_pos, y_pos, '0.0',  # odom at spawn position relative to world
                    '0.0', '0.0', yaw_pos,  # with yaw orientation
                    'world', 'odom'
                ],
                output='screen',
            )
        ]
    )
    
    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(set_env)
    ld.add_action(world_name_arg)
    ld.add_action(x_pos_arg)
    ld.add_action(y_pos_arg)
    ld.add_action(z_pos_arg)
    ld.add_action(yaw_pos_arg)
    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(world_to_odom_transform)
    ld.add_action(rviz_node)

    return ld
