# RC Car Simulation

ROS 2 and Gazebo simulation workspace for an RC car, including the vehicle model,
custom map, sensors, traffic lights, and supporting plugins.

## Build and run

```bash
./build.sh
source install/setup.bash
ros2 launch sim_pkg simulation_environment.launch.py
```

Docker helpers are also available through `docker-compose.yml` and `run.sh`.
