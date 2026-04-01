# slambot_bringup

Top-level bringup launch package for Slambot.

## Main launch

```bash
ros2 launch slambot_bringup slambot_bringup.launch.py sim:=false localization_mode:=true
```

## Key launch arguments

- `sim`
- `localization_mode`
- `map_yaml`
- `imu_input_topic`

Default map selection:

- `sim:=true` -> `/home/slambot/ros2_ws/maps/my_map.yaml`
- `sim:=false` -> `/home/slambot/ros2_ws_pi/maps/my_map_real.yaml`
