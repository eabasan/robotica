import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, EmitEvent, RegisterEventHandler, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # Set environment variable
    set_env = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')

    # Declare launch arguments
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(get_package_share_directory('localization'), 'maps', 'map.yaml'),
        description='Full path to map yaml file'
    )

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='scan',
        description='Scan topic'
    )

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='turtlebot3_dqn_stage4.world',
        description='Name of the Gazebo world'
    )

    rviz_conf_arg = DeclareLaunchArgument(
        'rviz_conf',
        default_value=os.path.join(get_package_share_directory('localization'), 'config', 'amcl.rviz'),
        description='RViz config file'
    )

    initial_pose_x_arg = DeclareLaunchArgument(
        'initial_pose_x',
        default_value='0.0',
        description='Initial pose x position'
    )

    initial_pose_y_arg = DeclareLaunchArgument(
        'initial_pose_y',
        default_value='0.0',
        description='Initial pose y position'
    )

    initial_pose_z_arg = DeclareLaunchArgument(
        'initial_pose_z',
        default_value='0.0',
        description='Initial pose z position'
    )

    initial_pose_a_arg = DeclareLaunchArgument(
        'initial_pose_a',
        default_value='0.0',
        description='Initial pose yaw angle'
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

    # Map server node
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map_file')}]
    )

    # AMCL node
    amcl_node = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'odom_model_type': 'diff'},
            {'odom_alpha5': 0.1},
            {'gui_publish_rate': 10.0},
            {'laser_max_beams': 60},
            {'laser_max_range': 12.0},
            {'min_particles': 500},
            {'max_particles': 2000},
            {'kld_err': 0.05},
            {'kld_z': 0.99},
            {'odom_alpha1': 0.2},
            {'odom_alpha2': 0.2},
            {'odom_alpha3': 0.2},
            {'odom_alpha4': 0.2},
            {'laser_z_hit': 0.5},
            {'laser_z_short': 0.05},
            {'laser_z_max': 0.05},
            {'laser_z_rand': 0.5},
            {'laser_sigma_hit': 0.2},
            {'laser_lambda_short': 0.1},
            {'laser_model_type': 'likelihood_field'},
            {'laser_likelihood_max_dist': 2.0},
            {'update_min_d': 0.25},
            {'update_min_a': 0.2},
            {'odom_frame_id': 'odom'},
            {'resample_interval': 1},
            {'transform_tolerance': 1.0},
            {'recovery_alpha_slow': 0.0},
            {'recovery_alpha_fast': 0.0},
            {'initial_pose_x': LaunchConfiguration('initial_pose_x')},
            {'initial_pose_y': LaunchConfiguration('initial_pose_y')},
            {'initial_pose_z': LaunchConfiguration('initial_pose_z')},
            {'initial_pose_a': LaunchConfiguration('initial_pose_a')}
        ],
        remappings=[
            ('scan', LaunchConfiguration('scan_topic'))
        ]
    )


    # Lifecycle transitions for map_server
    configure_map_server = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(map_server_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_map_server = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(map_server_node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    # Lifecycle transitions for amcl
    configure_amcl = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(amcl_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_amcl = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(amcl_node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_conf')],
        name='rviz2'
    )

    # Publish initial pose after AMCL is active
    initial_pose_publisher = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once', '/initialpose',
                    'geometry_msgs/msg/PoseWithCovarianceStamped',
                    '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]}}'
                ],
                output='screen'
            )
        ]
    )

    # rviz = TimerAction(
    #     period=10.0,
    #     actions=[rviz_node]
    # )

    return LaunchDescription([
        set_env,
        map_file_arg,
        scan_topic_arg,
        world_name_arg,
        rviz_conf_arg,
        rviz_node,
        initial_pose_x_arg,
        initial_pose_y_arg,
        initial_pose_z_arg,
        initial_pose_a_arg,
        turtlebot_sim_launch,
        map_server_node,
        amcl_node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=amcl_node,
                on_start=[configure_amcl, activate_amcl],
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=map_server_node,
                on_start=[configure_map_server, activate_map_server],
            )
        ),
        initial_pose_publisher,
    ])