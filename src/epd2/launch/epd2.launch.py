import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Ruta al simulador de TurtleBot3 [cite: 108-113]
    pkg_gazebo = get_package_share_directory('turtlebot3_gazebo')
    
    # 2. Declaramos los argumentos (lo que verás en la terminal) [cite: 171-172, 192]
    declare_lin_vel = DeclareLaunchArgument(
        'lin_vel', default_value='0.3', 
        description='Velocidad lineal máxima del robot'
    )
    declare_ang_vel = DeclareLaunchArgument(
        'ang_vel', default_value='0.5', 
        description='Velocidad angular máxima del robot'
    )

    # 3. Configuramos tu nodo controlCollisionCheck de la EPD 2 [cite: 174-182, 199, 211]
    node_collision = Node(
        package='epd2',
        executable='controlCollisionCheck',
        name='controlCollisionCheck',
        output='screen',
        parameters=[{
            'max_lin_vel': LaunchConfiguration('lin_vel'),
            'max_ang_vel': LaunchConfiguration('ang_vel')
        }]
    )

    # 4. Incluimos el lanzamiento de Gazebo (empty_world) [cite: 114-118, 205]
    include_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'empty_world.launch.py')
        )
    )

    # 5. Devolvemos la descripción para que ROS ejecute todo [cite: 131, 184-188]
    return LaunchDescription([
        declare_lin_vel,
        declare_ang_vel,
        include_gazebo,
        node_collision
    ])