from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    file_name_arg = DeclareLaunchArgument(
        'path_file_name',
        default_value='path.yaml',
        description='Name of the path configuration file (with .yaml extension)'
    )

    # Path to the included launch file
    #file_path = os.path.join(
    #    get_package_share_directory('turtlebot3_gazebo'),
    #    'config',
    #    LaunchConfiguration('path_file_name')
    #)
    package_share = get_package_share_directory('robot_controller')
    file_path = PathJoinSubstitution([
                package_share,
                'config',
                LaunchConfiguration('path_file_name')
            ])
    
    path_publisher_node = Node(
        package='robot_controller',
        executable='path_publisher',
        name='path_publisher',
        output='screen',
        parameters=[
            {'path_file': file_path},
        ]
    )
    
    return LaunchDescription([
        file_name_arg,
        path_publisher_node,
    ])
