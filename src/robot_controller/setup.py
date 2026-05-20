from setuptools import find_packages, setup

package_name = 'robot_controller'

# Include all launch files, config files, and model files in the package
import glob
launch_files = glob.glob('launch/*.py') 
config_files = glob.glob('config/*.yaml') + glob.glob('config/*.rviz')
world_files = glob.glob('worlds/*.sdf') + glob.glob('worlds/*.world')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', launch_files),
        ('share/' + package_name + '/config', config_files),
        ('share/' + package_name + '/worlds', world_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='The robot_controller package for TurtleBot3 path following and collision avoidance',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = robot_controller.robot_controller:main',
            'path_publisher = robot_controller.path_publisher:main',
        ],
    },
)
