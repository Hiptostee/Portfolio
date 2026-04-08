# paesano_localization

Localization utilities for Paesano: IMU covariance adapter, particle filter, and map load bridge.

## Config

- node config: `config/localization.yaml`
- EKF config: `config/ekf.yaml`

## Default interfaces

- estimated pose: `/estimated_pose`
- map: `/map`
- navigation state: `/is_navigating`
- load-map service: `/paesano/load_map`

## Launch

```bash
ros2 launch paesano_localization paesano_localization.launch.py localization_mode:=true sim:=false
```

## Load a map

Sim:

```bash
ros2 service call /paesano/load_map nav2_msgs/srv/LoadMap "{map_url: '/home/Paesano/ros2_ws/maps/my_map.yaml'}"
```

Real robot:

```bash
ros2 service call /paesano/load_map nav2_msgs/srv/LoadMap "{map_url: '/home/Paesano/ros2_ws_pi/maps/my_map_real.yaml'}"
```
