# slambot_description

Robot description, simulation assets, Gazebo bridge config, and kinematic base plugin for Slambot.

## Main launch

```bash
ros2 launch slambot_description slambot_description.launch.py sim:=true localization_mode:=false
```

## Important assets

- robot model: `urdf/slambot.xacro`
- bridge config: `config/bridge.yaml`
- world: `worlds/mecanum_ode.sdf`
