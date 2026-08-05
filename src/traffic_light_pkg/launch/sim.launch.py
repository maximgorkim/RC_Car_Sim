#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def spawn_model(models_path, model_type, entity, x, y, z, yaw, delay):
    return TimerAction(period=delay, actions=[Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-file', f'{models_path}/{model_type}/model.sdf', '-entity', entity,
                  '-x', str(x), '-y', str(y), '-z', str(z), '-Y', str(yaw)],
        output='screen'
    )])

def generate_launch_description():
    world_file = os.path.expanduser('~/bfmc_sim/src/sim_pkg/worlds/world_with_separators.world')
    models_path = os.path.expanduser('~/bfmc_sim/src/models_pkg')
    install_path = os.path.expanduser('~/bfmc_sim/install')
    gazebo_launch = '/opt/ros/humble/share/gazebo_ros/launch/gazebo.launch.py'
    
    os.environ['GAZEBO_MODEL_PATH'] = f"{models_path}:{os.environ.get('GAZEBO_MODEL_PATH', '')}"
    os.environ['GAZEBO_RESOURCE_PATH'] = f"/usr/share/gazebo-11:{os.environ.get('GAZEBO_RESOURCE_PATH', '')}"
    plugin_paths = [f"{install_path}/car_plugin/lib", f"{install_path}/bno055_plugin/lib",
                   f"{install_path}/gps_plugin/lib", f"{install_path}/traffic_light_plugin/lib",
                   f"{install_path}/pedestrian_object_plugin/lib"]
    os.environ['GAZEBO_PLUGIN_PATH'] = ':'.join(plugin_paths + [os.environ.get('GAZEBO_PLUGIN_PATH', '')])
    
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={'world': world_file, 'verbose': 'false',
                         'gui': LaunchConfiguration('gui'), 'pause': 'false'}.items())
    
    tl_node = TimerAction(period=3.0, actions=[Node(package='traffic_light_pkg', executable='tl_talker',
                                                     name='traffic_light_publisher_node', output='screen')])
    
    # Static TF publisher for sensor frames
    tf_node = TimerAction(period=6.0, actions=[Node(package='tf_publisher_pkg', executable='static_tf_publisher',
                                                     name='static_tf_publisher', output='screen')])
    
    car = spawn_model(models_path, 'rcCar_assembly', 'automobile', 0.82, -14.91, 0.032939, 1.570796, 5.0)
    tl1 = spawn_model(models_path, 'traffic_light', 'start', 1.132, -14.403, 0, 4.712388, 6.0)
    tl2 = spawn_model(models_path, 'traffic_light', 'master', 2.030, -11.132, 0, 3.141592, 6.3)
    tl3 = spawn_model(models_path, 'traffic_light', 'slave', 3.744, -10.140, 0, 0, 6.6)
    tl4 = spawn_model(models_path, 'traffic_light', 'antimaster', 3.347, -11.483, 0, 4.712388, 6.9)
    
    eh1 = spawn_model(models_path, 'enter_highway_sign', 'EHIGH_T', 5.871, -13.970, 0.078998, 3.141592, 7.2)
    eh2 = spawn_model(models_path, 'enter_highway_sign', 'EHIGH_Y', 9.412, -4.850, 0.078998, 1.570796, 7.5)
    lh1 = spawn_model(models_path, 'leave_highway_sign', 'LHIGH_U', 10.686, -6.276, 0.078998, 4.712388, 7.8)
    lh2 = spawn_model(models_path, 'leave_highway_sign', 'LHIGH_Z', 6.520, -12.700, 0.078998, 0, 8.1)
    lh3 = spawn_model(models_path, 'leave_highway_sign', 'LHIGH_AA', 7.302, -13.971, 0.078998, 3.141592, 8.4)
    s1 = spawn_model(models_path, 'stop_sign', 'STOP_A', 2.103, -14.004, 0.079, 0, 8.7)
    s2 = spawn_model(models_path, 'stop_sign', 'STOP_C', 4.272, -11.153, 0.078998, 3.141592, 9.0)
    s3 = spawn_model(models_path, 'stop_sign', 'STOP_E', 1.135, -6.018, 0.078998, 4.712388, 9.3)
    s4 = spawn_model(models_path, 'stop_sign', 'STOP_G', 6.958, -9.804, 0.078998, 1.570796, 9.6)
    s5 = spawn_model(models_path, 'stop_sign', 'STOP_I', 4.942, -6.006, 0.078998, 4.712388, 9.9)
    p1 = spawn_model(models_path, 'priority_sign', 'PRIOR_D', 3.675, -13.050, 0.078998, 3.141592, 10.2)
    p2 = spawn_model(models_path, 'priority_sign', 'PRIOR_F', 0.204, -6.031, 0.078998, 4.712388, 10.5)
    p3 = spawn_model(models_path, 'priority_sign', 'PRIOR_H', 7.888, -9.820, 0.078998, 1.570796, 10.8)
    p4 = spawn_model(models_path, 'priority_sign', 'PRIOR_M', 3.014, -6.016, 0.078998, 4.712388, 11.1)
    p5 = spawn_model(models_path, 'priority_sign', 'PRIOR_O', 7.896, -7.562, 0.078998, 1.570796, 11.4)
    c1 = spawn_model(models_path, 'crosswalk_sign', 'CWALK_K', 7.312, -3.240, 0.078998, 3.141592, 11.7)
    c2 = spawn_model(models_path, 'crosswalk_sign', 'CWALK_L', 6.627, -4.181, 0.078998, 0, 12.0)
    c3 = spawn_model(models_path, 'crosswalk_sign', 'CWALK_N', 1.088, -8.690, 0.078998, 4.712388, 12.3)
    pk1 = spawn_model(models_path, 'parking_sign', 'PRK_P1', 4.047, -2.358, 0.078998, 0, 12.6)
    pk2 = spawn_model(models_path, 'parking_sign', 'PRK_P2', 2.857, -2.358, 0.078998, 0, 12.9)
    pk3 = spawn_model(models_path, 'parking_sign', 'PRK_P3', 1.681, -2.358, 0.078998, 0, 13.2)
    ow1 = spawn_model(models_path, 'oneway_sign', 'ONEWAY_J', 3.341, -9.821, 0.078998, 1.570796, 13.5)
    ow2 = spawn_model(models_path, 'oneway_sign', 'ONEWAY_B', 2.406, -9.821, 0.078998, 1.570796, 13.8)
    pr1 = spawn_model(models_path, 'prohibited_sign', 'DENY_V', 3.323, -7.581, 0.078998, 4.712388, 14.1)
    pr2 = spawn_model(models_path, 'prohibited_sign', 'DENY_X', 2.387, -7.581, 0.078998, 4.712388, 14.4)
    pr3 = spawn_model(models_path, 'prohibited_sign', 'DENY_W', 8.838, -7.584, 0.078998, 1.570796, 14.7)
    r1 = spawn_model(models_path, 'roundabout_sign', 'GIR_Q', 8.766, -4.179, 0.078998, 0, 15.0)
    r2 = spawn_model(models_path, 'roundabout_sign', 'GIR_R', 10.320, -4.869, 0.078998, 1.570796, 15.3)
    r3 = spawn_model(models_path, 'roundabout_sign', 'GIR_S', 9.430, -3.290, 0.078998, 3.141592, 15.6)
    ped1 = spawn_model(models_path, 'pedestrian_object', 'PED_OBJ_KL', 6.427, -4.181, 0, 0, 15.9)
    ped2 = spawn_model(models_path, 'pedestrian_object', 'PED_OBJ_MN', 1.998, -6.320, 0, 4.712388, 16.2)
    ped3 = spawn_model(models_path, 'pedestrian_object', 'PED_OBJ_OP', 1.088, -8.698, 0, 1.570796, 16.5)
    obs1 = spawn_model(models_path, 'rcCar_assembly_obstacle', 'obstacle_rcCar_1', 3.703, -2.580, 0.032939, 1.567, 16.8)
    obs2 = spawn_model(models_path, 'rcCar_assembly_obstacle', 'obstacle_rcCar_2', 3.999, -1.376, 0.032939, 3.137, 17.1)
    obs3 = spawn_model(models_path, 'rcCar_assembly_obstacle', 'obstacle_rcCar_3', 2.817, -1.376, 0.032939, 3.137, 17.4)
    ramp = spawn_model(models_path, 'ramp', 'RAMP', 0.651761, -4.561203, 0.0, 1.569651, 17.7)
    rb1 = spawn_model(models_path, 'roadblock', 'roadblock_1', 3.054809, -8.697316, 0.0, -1.569651, 18.0)
    rb2 = spawn_model(models_path, 'roadblock', 'roadblock_2', 2.682368, -8.696889, 0.0, 1.569651, 18.3)
    
    return LaunchDescription([gui_arg, gazebo, tl_node, tf_node, car, tl1, tl2, tl3, tl4,
        eh1, eh2, lh1, lh2, lh3, s1, s2, s3, s4, s5, p1, p2, p3, p4, p5,
        c1, c2, c3, pk1, pk2, pk3, ow1, ow2, pr1, pr2, pr3, r1, r2, r3,
        ped1, ped2, ped3, obs1, obs2, obs3, ramp, rb1, rb2])
