from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument # Para crear el argumento
from launch.substitutions import LaunchConfiguration # Para leer su valor

def generate_launch_description():
    # 1. Ruta al simulador original de TurtleBot3
    included_launch_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch',
        'empty_world.launch.py'
    )
    # Declaramos el argumento que el usuario podrá escribir en la terminal
    declare_linvel_arg = DeclareLaunchArgument(
        'max_linear_velocity', 
        default_value='0.4' # Valor por defecto si el usuario no pone nada
    )

    # 2. Acción para incluir el simulador
    include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(included_launch_path)
    )

    # 3. Definición de tu nodo (el que creamos en la EPD 1)
    delayed_node = Node(
        package='epd1',
        executable='forward_node',
        name='forward_node',
        output='screen',
        # Aquí es donde vinculamos el argumento del launch con el parámetro del nodo
        parameters=[
            {'max_linear_velocity': LaunchConfiguration('max_linear_velocity')}
        ]
    )

    # 4. Temporizador: esperar 10 segundos a que Gazebo cargue bien
    delayed_node_action = TimerAction(
        period=10.0,
        actions=[delayed_node]
    )

    return LaunchDescription([
        declare_linvel_arg, #añadido
        include_launch,
        delayed_node_action
    ])
