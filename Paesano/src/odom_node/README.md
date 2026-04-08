# odom_node

Wheel-encoder odometry node for Paesano.

## Node

- executable: `odom_node_exec`
- config: `config/odom.yaml`

## Default interfaces

- encoder input: `/wheel_encoders`
- odometry output: `/odom`

## Launch

```bash
ros2 launch odom_node odom_node.launch.py sim:=false
```
