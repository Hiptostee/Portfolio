# slambot_mapping

Mapping launch and `slam_toolbox` config for Slambot.

## Config

- slam config: `config/slam_toolbox.yaml`

## Default interface

- map topic: `/map`

## Save a map

```bash
ros2 run nav2_map_server map_saver_cli -f /home/slambot/ros2_ws/maps/my_map
```
