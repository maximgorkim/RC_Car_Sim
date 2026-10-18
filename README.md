# RC Car Simulation

ROS 2 and Gazebo simulation workspace for an RC car, including the vehicle model,
custom map, sensors, traffic lights, and supporting plugins.

> **Fork of [memre12/RC_Car_Sim](https://github.com/memre12/RC_Car_Sim)** by
> Mehmet Emre Demir ([@memre12](https://github.com/memre12)), who built the
> simulation and its car-like mode. This fork only adds an optional
> `clock_rate` launch argument (branch `car-like-mode`): Gazebo's default 10 Hz
> `/clock` quantizes every sensor stamp to 0.1 s, which breaks the EKF's delay
> compensation in [RC_Car_Codes](https://github.com/maximgorkim/RC_Car_Codes/tree/feature/ekf-localization).
>
> ```bash
> ros2 launch sim_pkg simulation_environment.launch.py vehicle:=car clock_rate:=1000
> ```

## Build and run

```bash
./build.sh
source install/setup.bash
ros2 launch sim_pkg simulation_environment.launch.py
```

Docker helpers are also available through `docker-compose.yml` and `run.sh`.

## Car-like mode (`vehicle:=car`)

```bash
ros2 launch sim_pkg simulation_environment.launch.py vehicle:=car [gui:=false] [car_like_config:=<file>]
```

`car_plugin` then reproduces the real car at its topic level, with every
parameter identified from the car's rosbags (`src/plugins_pkgs/car_plugin/config/car_like.json`;
sources and numbers inside; identification and validation in RC_Car_Codes,
README "Gazebo car-like mode"):

- commands in the car's units (`/commands/servo/position`, `/commands/motor/speed`)
  through the car's dead times, lags and measured steering map, plus the car's
  measured yaw disturbance;
- `/pf/pose/odom` like the car's particle filter: 8.44 Hz with scan and
  publication jitter, 0.2 s latency, 2.6° heading bias, measured lateral /
  longitudinal / heading noise, no twist;
- `/zed2/zed_node/imu/data` (197 Hz, measured yaw-rate noise), the VESC's ERPM on
  `/sim/vesc/erpm` (RC_Car_Codes' `sim_vesc_bridge` publishes `/sensors/core`),
  and the true pose on `/sim/ground_truth/odom` for evaluation.

`vehicle:=ideal` (default) is the original model, unchanged.
