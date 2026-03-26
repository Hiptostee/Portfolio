# slambot_localization

Localization utilities for Slambot: IMU covariance adapter, particle filter, and map load bridge.

## Config

- node config: `config/localization.yaml`
- EKF config: `config/ekf.yaml`

## Default interfaces

- estimated pose: `/estimated_pose`
- map: `/map`
- navigation state: `/is_navigating`
- load-map service: `/slambot/load_map`

## Launch

```bash
ros2 launch slambot_localization slambot_localization.launch.py localization_mode:=true sim:=false
```

## Load a map

Use the Slambot workspace map path.

```bash
ros2 service call /slambot/load_map nav2_msgs/srv/LoadMap "{map_url: '/home/slambot/ros2_ws/maps/my_map.yaml'}"
```
