# paesano_explorer

Composable frontier-exploration node for Paesano. The ROS flow is scaffolded while frontier
detection, clustering, and goal selection remain for Joseph to implement in the `.cpp` files.

## Runtime

- Component: `paesano_explorer::ExplorerNode`
- Launch: `launch/explorer.launch.py`
- Config: `config/explorer.yaml`

Default subscriptions:

- `/map`
- `/estimated_pose`
- `/navigation/result`

Default publications:

- `/navigation/goal`
- `/exploration/state`
- `/exploration/frontiers`

The component is created only when `auto_explore:=true`.

```bash
ros2 launch paesano_explorer explorer.launch.py sim:=true auto_explore:=true
```
