# paesano_description

Robot description, simulation assets, Gazebo bridge config, and kinematic base plugin for Paesano.

## Main launch

```bash
ros2 launch paesano_description paesano_description.launch.py sim:=true localization_mode:=false
```

## Important assets

- robot model: `urdf/paesano.xacro`
- bridge config: `config/bridge.yaml`
- world: `worlds/mecanum_ode.sdf`
