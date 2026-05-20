import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, AppendEnvironmentVariable, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Set environment variables
    set_env_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')
    #set_env_gazebo = SetEnvironmentVariable('GAZEBO_RESOURCE_PATH',
    #    os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models', 'autorace', 'ground_picture'))

    # Declare launch arguments
    world_name_arg = DeclareLaunchArgument(
        'world_name_and_path',
        default_value='turtlebot3_stage_2',
        description='World name'
    )
    x_pos_arg = DeclareLaunchArgument(
        'x_pos',
        default_value='0.0',
        description='X position'
    )
    y_pos_arg = DeclareLaunchArgument(
        'y_pos',
        default_value='0.0',
        description='Y position'
    )
    z_pos_arg = DeclareLaunchArgument(
        'z_pos',
        default_value='0.0',
        description='Z position'
    )
    yaw_pos_arg = DeclareLaunchArgument(
        'yaw_pos',
        default_value='0.0',
        description='Yaw orientation of the robot in radians'
    )

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
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

    # Static transform publishers
    # camera_dummy_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='camera_dummy_tf',
    #     arguments=['0.04', '0', '0.11', '0', '0', '0', 'base_link', 'camera_dummy_link']
    # )

    # camera_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='camera_tf',
    #     arguments=['0.04', '0', '0.11', '0', '0', '0', 'camera_dummy_link', 'camera_link']
    # )

    # camera_optical_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='camera_optical_tf',
    #     arguments=['0.04', '0', '0.11', '-1.57', '0', '-1.5707', 'camera_link', 'camera_optical_frame']
    # )

    # Get configuration values
    x_pos = LaunchConfiguration('x_pos')
    y_pos = LaunchConfiguration('y_pos')
    yaw_pos = LaunchConfiguration('yaw_pos')


    # Spawn model
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


    # # Fix TF hierarchy and positioning
    # # Publish world -> odom with translation (-2,0,0) so:
    # # - world is at (0,0) - center of environment
    # # - odom is at (-2,0) relative to world - spawn position
    # world_to_odom_transform = TimerAction(
    #     period=2.5,
    #     actions=[
    #         Node(
    #             package='tf2_ros',
    #             executable='static_transform_publisher',
    #             arguments=[
    #                 x_pos, y_pos, '0.0',  # odom at spawn position relative to world
    #                 '0.0', '0.0', yaw_pos,  # with yaw orientation
    #                 'world', 'odom'
    #             ],
    #             output='screen',
    #         )
    #     ]
    # )

    return LaunchDescription([
        set_env_model,
        world_name_arg,
        x_pos_arg,
        y_pos_arg,
        z_pos_arg,
        yaw_pos_arg,
        #rviz_conf_arg,
        set_env_vars_resources,
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher_cmd,
        #camera_dummy_tf,
        #camera_tf,
        #camera_optical_tf,
        spawn_turtlebot_cmd,
        #world_to_odom_transform,   
        #rviz_node
    ])