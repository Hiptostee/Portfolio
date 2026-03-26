# slambot_traj_following

Trajectory-following controller for Slambot.

## Node

- executable: `lqr`
- launch: `launch/lqr.launch.py`
- config: `config/lqr.yaml`

## Default interfaces

- estimated pose input: `/estimated_pose`
- path input: `/path`
- velocity output: `/cmd_vel`
- navigation state output: `/is_navigating`

## Launch

```bash
ros2 launch slambot_traj_following lqr.launch.py sim:=false
```
