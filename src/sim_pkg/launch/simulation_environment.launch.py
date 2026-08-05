#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import os

def generate_launch_description():
    # Paths
    sim_share = get_package_share_directory('sim_pkg')
    models_path = get_package_share_directory('models_pkg')
    install_path = os.path.dirname(get_package_prefix('sim_pkg'))
    world_file = os.path.join(sim_share, 'worlds', 'my_map_world.world')
    gazebo_launch = '/opt/ros/humble/share/gazebo_ros/launch/gazebo.launch.py'
    
    # Set Gazebo environment variables
    os.environ['GAZEBO_MODEL_PATH'] = f"{models_path}:{os.environ.get('GAZEBO_MODEL_PATH', '')}"
    os.environ['GAZEBO_RESOURCE_PATH'] = f"/usr/share/gazebo-11:{os.environ.get('GAZEBO_RESOURCE_PATH', '')}"
    
    # Plugin paths
    plugin_paths = [
        f"{install_path}/car_plugin/lib",
        f"{install_path}/bno055_plugin/lib",
        f"{install_path}/gps_plugin/lib"
    ]
    os.environ['GAZEBO_PLUGIN_PATH'] = ':'.join(plugin_paths + [os.environ.get('GAZEBO_PLUGIN_PATH', '')])
    
    # Launch arguments
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
            'gui': LaunchConfiguration('gui'),
            'pause': 'false'
        }.items()
    )
    
    # Spawn the car with a delay
    car = TimerAction(
        period=5.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', f'{models_path}/rcCar_assembly/model.sdf',
                '-entity', 'automobile',
                '-x', '7.163432',
                '-y', '-9.692866',
                '-z', '0.032939',
                '-Y', '-0.548857'
            ],
            output='screen'
        )]
    )
    
    return LaunchDescription([gui_arg, gazebo, car])
