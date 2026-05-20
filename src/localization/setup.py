from setuptools import find_packages, setup

package_name = 'localization'

# Include all launch files, config files, and map files in the package
import glob
launch_files = glob.glob('launch/*.py')
config_files = glob.glob('config/*.rviz')
map_files = glob.glob('maps/*')

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
        ('share/' + package_name + '/maps', map_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='The localization package',
    license='TODO',
    tests_require=['pytest'],
)