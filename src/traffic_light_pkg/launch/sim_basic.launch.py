#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Paths
    world_file = os.path.expanduser('~/bfmc_sim/src/sim_pkg/worlds/world_with_separators.world')
    models_path = os.path.expanduser('~/bfmc_sim/src/models_pkg')
    install_path = os.path.expanduser('~/bfmc_sim/install')
    gazebo_launch = '/opt/ros/humble/share/gazebo_ros/launch/gazebo.launch.py'
    
    # Set environment variables for Gazebo
    os.environ['GAZEBO_MODEL_PATH'] = f"{models_path}:{os.environ.get('GAZEBO_MODEL_PATH', '')}"
    os.environ['GAZEBO_RESOURCE_PATH'] = f"/usr/share/gazebo-11:{os.environ.get('GAZEBO_RESOURCE_PATH', '')}"
    
    # Add all plugin paths
    plugin_paths = [
        f"{install_path}/car_plugin/lib",
        f"{install_path}/bno055_plugin/lib",
        f"{install_path}/gps_plugin/lib",
        f"{install_path}/traffic_light_plugin/lib",
        f"{install_path}/pedestrian_object_plugin/lib"
    ]
    existing_plugin_path = os.environ.get('GAZEBO_PLUGIN_PATH', '')
    os.environ['GAZEBO_PLUGIN_PATH'] = ':'.join(plugin_paths + [existing_plugin_path])
    
    # Launch arguments
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    
    # Use gazebo_ros launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
            'gui': LaunchConfiguration('gui'),
            'pause': 'false'
        }.items()
    )
    
    # Traffic light controller
    traffic_light_node = TimerAction(
        period=3.0,
        actions=[Node(
            package='traffic_light_pkg',
            executable='tl_talker',
            name='traffic_light_publisher_node',
            output='screen'
        )]
    )
    
    # Spawn car
    spawn_car = TimerAction(
        period=5.0,
        actions=[Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', f'{models_path}/rcCar_assembly/model.sdf',
                      '-entity', 'automobile',
                      '-x', '0.82', '-y', '-14.91', '-z', '0.032939', '-Y', '1.570796'],
            output='screen'
        )]
    )
    
    # Spawn traffic lights
    spawn_tl_start = TimerAction(period=6.0, actions=[Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-file', f'{models_path}/traffic_light/model.sdf', '-entity', 'start',
                  '-x', '1.132', '-y', '-14.403', '-z', '0', '-Y', '4.712388'],
        output='screen'
    )])
    
    spawn_tl_master = TimerAction(period=7.0, actions=[Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-file', f'{models_path}/traffic_light/model.sdf', '-entity', 'master',
                  '-x', '2.030', '-y', '-11.132', '-z', '0', '-Y', '3.141592'],
        output='screen'
    )])
    
    spawn_tl_slave = TimerAction(period=8.0, actions=[Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-file', f'{models_path}/traffic_light/model.sdf', '-entity', 'slave',
                  '-x', '3.744', '-y', '-10.140', '-z', '0', '-Y', '0'],
        output='screen'
    )])
    
    spawn_tl_antimaster = TimerAction(period=9.0, actions=[Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-file', f'{models_path}/traffic_light/model.sdf', '-entity', 'antimaster',
                  '-x', '3.347', '-y', '-11.483', '-z', '0', '-Y', '4.712388'],
        output='screen'
    )])
    
    return LaunchDescription([
        gui_arg, gazebo, traffic_light_node,
        spawn_car, spawn_tl_start, spawn_tl_master, spawn_tl_slave, spawn_tl_antimaster
    ])
