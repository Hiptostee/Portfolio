# slambot_navigation

A\* path planning for Slambot.

## Node

- executable: `a_star_action_server`
- launch: `launch/a_star_action_server.launch.py`
- config: `config/navigation.yaml`

## Default interfaces

- action: `/a_star`
- map input: `/map`
- pose input: `/estimated_pose`
- path output: `/path`

## Send a goal

```bash
ros2 action send_goal /a_star slambot_navigation/action/AStar "{
  goal: {
    header: {frame_id: 'map'},
    pose: {
      position: {x: 2.0, y: -2.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}" --feedback
```
