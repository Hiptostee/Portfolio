# paesano_explorer

Composable frontier-exploration node for Paesano. It detects and clusters map frontiers, selects
navigation goals, and tracks the exploration lifecycle.

## Runtime

- Component: `paesano_explorer::ExplorerNode`
- Launch: `launch/explorer.launch.py`
- Config: `config/explorer.yaml`

Default subscriptions:

- `/map`
- `/map_inflated`
- `/estimated_pose`
- `/navigation/result`

Default publications:

- `/navigation/goal`
- `/exploration/state`
- `/exploration/frontiers`

Exploration states are `WAITING_FOR_DATA`, `SELECTING`, `NAVIGATING`, `STUCK`, and `COMPLETE`.
`STUCK` means retained frontiers exist but none currently has a safe, non-blacklisted approach;
new map data can make it selectable again. `COMPLETE` requires repeated map updates with no
retained frontier clusters.

The component is created only when `auto_explore:=true`.

```bash
ros2 launch paesano_explorer explorer.launch.py sim:=true auto_explore:=true
```
