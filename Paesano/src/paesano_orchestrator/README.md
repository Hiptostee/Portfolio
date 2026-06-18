# paesano_orchestrator

Central navigation coordinator for Paesano. Owns the full navigation lifecycle — receiving
goals, calling the A\* planner, monitoring the LQR controller, detecting dynamic obstacles
via a local costmap, and replanning when the path is blocked.

---

## Motivation

Without this module, goals flow from the mobile bridge directly to A\* and then to LQR with
no central state. LQR has no awareness of newly appeared obstacles and will drive the robot
into them. The orchestrator adds:

- A **state machine** so the system is always in exactly one well-defined mode
- A **dynamic local costmap** (`paesano_local_map`) built from live LiDAR readings
- Automatic **stop-and-replan** when the path ahead is obstructed
- A single readable `tick()` function that describes the robot's behaviour in plain C++

---

## Package Layout

```
src/paesano_local_map/          ← standalone library, no node
    include/paesano_local_map/
        local_map.hpp
    src/
        local_map.cpp
    CMakeLists.txt
    package.xml

src/paesano_orchestrator/       ← ROS2 component node
    include/paesano_orchestrator/
        orchestrator.hpp        # state enum, members, callback declarations
    src/
        orchestrator.cpp        # ← THE READABLE FILE: tick() + state transitions
        orchestrator_node.cpp   # rclcpp_components registration boilerplate
    CMakeLists.txt
    package.xml
    README.md                   # this file
```

---

## State Machine

```
          new goal received
IDLE ─────────────────────► PLANNING
 ▲                               │ A* fails
 │                               ├─────────────────────► IDLE
 │              path received    │
 │◄──────────────────────────── ▼
 │                          NAVIGATING
 │                               │ goal reached
 │                               ├─────────────────────► GOAL_REACHED ──► IDLE
 │                               │ local costmap: path blocked
 │                               ▼
 │                            BLOCKED
 │                               │ lqr_stop() sent, new A* goal sent
 │                               ▼
 │                          REPLANNING
 │ replan fails                  │ new path received
 ◄───────────────────────────── ▼
                            NAVIGATING
```

### States

| State | Meaning |
|---|---|
| `IDLE` | No active goal. Waiting for `/navigation/goal`. |
| `PLANNING` | A\* action in flight. Waiting for path result. |
| `NAVIGATING` | LQR is tracking the path. Checking local map each tick. |
| `BLOCKED` | Obstacle detected ahead. Stopping LQR and replanning. |
| `REPLANNING` | A\* action in flight with same destination, new start pose. |
| `GOAL_REACHED` | Robot within `goal_tolerance_m` of goal. Transitioning to IDLE. |

---

## The Tick Function (`orchestrator.cpp`)

The entire navigation policy lives here. Called every 100 ms.

```cpp
void Orchestrator::tick()
{
    switch (state_) {
    case State::IDLE:
        break;

    case State::PLANNING:
    case State::REPLANNING:
        // Async — result handled in goalResultCallback()
        break;

    case State::NAVIGATING:
        if (local_map_.isPathBlocked(current_path_, path_index_, lookahead_n_)) {
            lqr_stop();
            send_astar_goal(current_goal_);
            transition(State::REPLANNING);
        } else if (goalReached()) {
            lqr_stop();
            transition(State::GOAL_REACHED);
        }
        break;

    case State::GOAL_REACHED:
        RCLCPP_INFO(get_logger(), "Goal reached.");
        transition(State::IDLE);
        break;
    }
}
```

All async I/O (action callbacks, service calls, topic callbacks) is in `orchestrator.hpp`
private methods. `tick()` only reads state and calls named helpers.

---

## ROS Interfaces

### Subscriptions

| Topic | Type | Purpose |
|---|---|---|
| `/navigation/goal` | `geometry_msgs/PoseStamped` | Incoming navigation goal |
| `/estimated_pose` | `geometry_msgs/PoseStamped` | Current robot pose (from particle filter) |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR scan → feeds local costmap |
| `/is_navigating` | `std_msgs/Bool` | LQR tracking state |

### Publications

| Topic | Type | QoS | Purpose |
|---|---|---|---|
| `/path` | `nav_msgs/Path` | transient\_local, reliable | Path for LQR to follow |
| `/orchestrator/state` | `std_msgs/String` | default | Current state name (monitoring/debug) |

### Action Client

| Action | Type | Purpose |
|---|---|---|
| `/a_star` | `paesano_navigation/AStar` | Request paths from the A\* planner |

### Service Client

| Service | Type | Purpose |
|---|---|---|
| `/lqr/stop` | `std_srvs/Trigger` | Stop LQR tracking before replanning |

---

## Goal Input

The orchestrator subscribes to `/navigation/goal` (`geometry_msgs/PoseStamped`, depth 1).
Publish to this topic from any source to start navigation.

The mobile bridge (`bridge_node.py`) is updated to publish here instead of calling the
`/a_star` action directly. Pause / resume / cancel / emergency stop still call `/lqr/stop`
directly — no change needed there.

---

## paesano_local_map

A pure C++ library (no ROS node). The orchestrator holds a `LocalMap` instance and calls it
from the scan subscription callback and from `tick()`.

```cpp
class LocalMap {
public:
    LocalMap(double size_m, double resolution_m, double obstacle_radius_m);

    // Call on every /scan message
    void updateFromScan(const sensor_msgs::msg::LaserScan & scan,
                        const geometry_msgs::msg::TransformStamped & laser_to_map);

    // Call from tick() — checks next lookahead_n waypoints
    bool isPathBlocked(const nav_msgs::msg::Path & path,
                       std::size_t from_index,
                       int lookahead_n) const;
};
```

### How the local map works

1. On each scan, clear the grid and re-mark occupied cells from LiDAR ray endpoints
   (transformed to map frame via the provided TF). Inflate each hit by `obstacle_radius_m`.
2. `isPathBlocked` converts the next N path waypoints from world coordinates into grid cells
   and returns `true` if any fall in an occupied cell.

No static map involvement — this detects only dynamic obstacles not present when A\* planned.

### Parameters (on orchestrator node)

| Parameter | Default | Description |
|---|---|---|
| `local_map_size_m` | `6.0` | Grid side length in metres |
| `local_map_resolution_m` | `0.05` | Cell size in metres (120×120 cells at defaults) |
| `obstacle_radius_m` | `0.25` | Inflation radius around each LiDAR hit |
| `path_blocked_lookahead` | `5` | Number of upcoming waypoints to check |
| `goal_tolerance_m` | `0.15` | Distance threshold for GOAL\_REACHED |
| `tick_period_ms` | `100` | Orchestrator tick rate |

---

## Bringup

Add the orchestrator component to the ROS2 component container in
`src/paesano_bringup/launch/paesano_bringup.launch.py`. It runs in the same process as
the other components (A\*, LQR, particle filter) with no special lifecycle requirements.

---

## Data Flow (updated)

```
Mobile App
  │  POST /navigation/goal
  ▼
Mobile Bridge  ──publishes──►  /navigation/goal
                                      │
                                      ▼
                               Orchestrator (IDLE → PLANNING)
                                      │  async_send_goal
                                      ▼
                               A* Action Server  ──► /path (transient_local)
                                                          │
                                      ◄────────────────────
                               Orchestrator (PLANNING → NAVIGATING)
                               monitors /scan via LocalMap
                                      │  if blocked: lqr_stop + replan
                                      │  if reached: lqr_stop
                                      ▼
                               LQR Controller  ──► /cmd_vel
                                      │
                                      ▼
                               Mecanum Drive  ──► Robot moves
```
