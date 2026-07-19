# paesano_mapping

Launches `slam_toolbox` and, during autonomous exploration, a composable TF pose publisher.

## Runtime

- Component: `paesano_mapping::MappingPosePublisher`
- Launch: `launch/mapping.launch.py`
- Config: `config/slam_toolbox.yaml`, `config/mapping_pose.yaml`

Default publications:

- `/map` from `slam_toolbox`
- `/estimated_pose` from `map -> base_link` TF when `auto_explore:=true`

```bash
ros2 launch paesano_mapping mapping.launch.py sim:=true auto_explore:=true
```
