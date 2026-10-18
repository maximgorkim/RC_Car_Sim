#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, TimerAction, DeclareLaunchArgument, OpaqueFunction,
                            SetEnvironmentVariable, UnsetEnvironmentVariable)
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
    # vehicle:=car -> car_plugin's car-like mode: the real car's identified actuators and
    # sensors (car_plugin/config/car_like.json, sources inside); vehicle:=ideal -> the
    # original model. The plugin reads CAR_LIKE_CONFIG from gzserver's environment.
    vehicle_arg = DeclareLaunchArgument('vehicle', default_value='ideal', description='ideal | car')
    car_like_arg = DeclareLaunchArgument(
        'car_like_config',
        default_value=os.path.join(get_package_share_directory('car_plugin'), 'config', 'car_like.json'),
        description='vehicle:=car only: car-like parameter file')

    def vehicle_env(context):
        vehicle = LaunchConfiguration('vehicle').perform(context)
        if vehicle == 'car':
            return [SetEnvironmentVariable('CAR_LIKE_CONFIG', LaunchConfiguration('car_like_config').perform(context))]
        if vehicle == 'ideal':
            return [UnsetEnvironmentVariable('CAR_LIKE_CONFIG')]
        raise RuntimeError(f"vehicle:={vehicle}: expected 'ideal' or 'car'")
    
    # clock_rate:=<Hz> -> /clock published at that rate (gazebo_ros default: 10 Hz). Every
    # plugin stamps its messages with the ROS clock, so with 10 Hz all sensor stamps are
    # quantized to 0.1 s (e.g. 95 % of the 200 Hz IMU messages share a stamp); timing-
    # sensitive consumers (EKF delay compensation, controller latency models) need ~1000.
    clock_rate_arg = DeclareLaunchArgument('clock_rate', default_value='',
                                           description='Hz of /clock; empty = gazebo_ros default (10)')

    def gazebo_include(context):
        args = {'world': world_file, 'verbose': 'false', 'gui': LaunchConfiguration('gui'), 'pause': 'false'}
        rate = LaunchConfiguration('clock_rate').perform(context)
        if rate:
            params = f'/tmp/gazebo_clock_{rate}.yaml'
            with open(params, 'w') as f:
                f.write(f'gazebo:\n  ros__parameters:\n    publish_rate: {float(rate)}\n')
            args['params_file'] = params
        return [IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch), launch_arguments=args.items())]
    
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
    
    return LaunchDescription([gui_arg, vehicle_arg, car_like_arg, clock_rate_arg, OpaqueFunction(function=vehicle_env),
                              OpaqueFunction(function=gazebo_include), car])
