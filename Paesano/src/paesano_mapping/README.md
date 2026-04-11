# paesano_mapping

Mapping launch and `slam_toolbox` config for Paesano.

## Config

- slam config: `config/slam_toolbox.yaml`

## Default interface

- map topic: `/map`

## Save a map

```bash
ros2 run nav2_map_server map_saver_cli -f /home/Paesano/ros2_ws/maps/my_map
```

ros2 run nav2_map_server map_saver_cli -f /home/paesano/ros2_ws_pi/maps/my_map_real
