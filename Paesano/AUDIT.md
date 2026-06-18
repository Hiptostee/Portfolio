# Paesano Codebase Audit

Scope: `paesano_firmware/`, `src/odom_node/`, `src/mecanum_drive_controller/`,
`src/paesano_localization/`, `src/paesano_navigation/`, `src/paesano_traj_following/`

Excluded: `ldlidar_driver/` (vendor), `bno08x_driver/sh2/` (vendor), Gazebo plugin.

---

## Summary

| Module | HIGH | MEDIUM | LOW | Total |
|---|---|---|---|---|
| paesano_firmware | 1 | 10 | 6 | 17 |
| odom_node | 1 | 3 | 2 | 6 |
| mecanum_drive_controller | 2 | 4 | 3 | 9 |
| paesano_localization | 4 | 10 | 1 | 15 |
| paesano_navigation | 4 | 8 | 4 | 16 |
| paesano_traj_following | 0 | 5 | 2 | 7 |
| **Total** | **12** | **40** | **18** | **70** |

---

## Critical Issues (HIGH)

### 1. `paesano_firmware/` — Missing `config.hpp`
Every firmware `.cpp` file `#include "config.hpp"` but the file does not exist in the repository. All compile-time constants (`DT`, `CONTROL_MS`, `MAX_TICKS_S`, encoder pin assignments, I2C address, etc.) are defined there. The firmware is **uncompilable** as checked-in. Either it is gitignored by mistake or was never committed.

### 2. `mecanum_drive_controller.cpp:77-81` — Fatal log without halt
`RCLCPP_FATAL` is called when `open()` or `ioctl()` fails, but execution continues normally. All subsequent I2C reads and writes use the invalid (negative) file descriptor. Should throw an exception or call `rclcpp::shutdown()` after logging.

### 3. `mecanum_drive_controller.hpp:46` — Uninitialized `i2c_file_`
`int i2c_file_` has no default member initializer (every other member in the class does). If the constructor throws before reaching `open()` on line 76, the destructor checks `if (i2c_file_ >= 0)` on an indeterminate value. Fix: `int i2c_file_{-1}`.

### 4. `a_star_action_server.cpp:98-100` — Detached thread holds `this`
```cpp
std::thread(std::bind(&AStarActionServer::execute, this, ...), goal_handle).detach();
```
The thread captures `this`. If the node is destroyed while a plan is in progress, the thread dereferences freed memory. Use a managed thread or join/cancel on destruction. The full scope of the threading problem is in §7 below.

### 5. `particle_filter.cpp:326-336` — `free_cells_` never cleared before rebuild
`globalLocalization()` appends to `free_cells_` without clearing it first. Each invocation (e.g., each time the robot gets lost and recovery is triggered) accumulates another full copy of the map's free cells on top of the previous. Memory grows without bound and sampling becomes biased toward whichever maps were loaded first.

### 6. `odom_node.cpp:32` — No guard on `ticks_per_rev` division
`distance_per_tick_ = (2.0 * M_PI * wheel_radius_) / ticks_per_rev;` uses the raw parameter value with no guard. A misconfigured launch file (e.g., `ticks_per_rev: 0`) produces `inf` for `distance_per_tick_`, silently corrupting all odometry. `mecanum_drive_controller` already does `std::max(ticks_per_rev_, 1e-6)` — apply the same here.

### 7. `a_star_action_server.cpp` — Data races during normal operation
The detached `execute()` thread races with the main executor on multiple member variables with no synchronization whatsoever:

- `have_map_` and `have_pose_` — read by `execute()`, written by `handleMap()` and `handlePose()` on the executor thread
- `current_pose_` — a multi-field `PoseStamped` struct read by `execute()` while `handlePose()` can write it mid-plan; a torn read produces a pose with mismatched position and orientation
- `planner_` — `execute()` calls `planner_.plan()` while the executor thread can call `planner_.setMap()` via `handleMap()`; the planner holds the full occupancy grid and inflated map

These are not just teardown edge cases. They fire during any normal planning request if a pose or map update arrives concurrently. Undefined behavior under the C++ memory model. Fix requires a mutex guarding all shared members, or restructuring so `execute()` snapshots everything it needs before starting.

### 8. `pf_compute_score.cpp:203` — `logw` vector allocated on every scan
`std::vector<double> logw(particles_.size(), 0.0)` is allocated fresh inside `score()` on every LiDAR scan (~10 Hz). With 250 particles this is ~2.5k heap allocations per second, causing allocator churn and heap fragmentation. Promote `logw` to a member variable and resize once at initialization.

### 9. `pf_compute_score.cpp:240-241` — Transcendental ops in the particle scoring hot loop
The log-sum-exp per beam per particle: `m + std::log(std::exp(log_p_hit - m) + std::exp(log_p_rand - m))` calls two `std::exp` and one `std::log` per beam. With 250 particles × ~10 beams per scan (stride=3) × 10 Hz = ~25,000 transcendental ops/sec. On ARM Cortex-A (RPi5), each is 20–50 cycles. Consider approximating or restructuring: since `z_rand_` contributes a fixed uniform term, `log_p_rand` is a compile-time-constant once parameters are set and can be precomputed outside the loop.

### 10. `pf_compute_score.cpp:101-108` — ~16 MB of temporary vectors on every distance field rebuild
`rebuildDistanceField()` allocates `col_sq_dist`, `row_sq_dist`, `f`, `d`, `v`, `z` vectors each of `cell_count` doubles. On a 200×200 cell map that is ~4 MB per vector × 4 vectors = ~16 MB of temporaries allocated and immediately discarded. Promote these to member variables and reuse them; or at minimum, call `reserve()` on them once.

### 11. `a_star.cpp:81-89` — ~28 MB of vectors allocated per `plan()` call
Five large vectors (`closed`, `came_from`, `g_score`, `h_score`, `f_score`) are allocated fresh on every planning request. On a 1000×1000 occupancy grid this totals ~28 MB per call. On repeated planning these allocations cause major TLB pressure and cache eviction on the RPi5 which has no L3 cache. Make them member variables of `AStarPlanner`, size them once in `setMap()`, and clear/reset them at the start of each `plan()`.

### 12. `a_star_helpers.cpp:193-195, 226` — Catmull-Rom parameterisation recomputed 100× per segment
`catmullRom()` is called 100 times per path segment from `applySpline()` (line 226). Each call recomputes `t0`, `t1`, `t2`, `t3` from scratch (lines 193–195), which involves 3 `std::pow` calls and 4 `distance()` (`std::hypot`) calls. For a 5-segment path this is 1,500 `hypot` + 1,500 `pow` operations for work that only needs to happen once per segment. Hoist `t0`–`t3` outside the sampling loop.

---

## `paesano_firmware/`

### `control.cpp`

| Line | Severity | Category | Issue |
|---|---|---|---|
| 22 | LOW | Concurrency | `hold_reset_req` is read unguarded on line 22 then cleared inside `noInterrupts` on line 25; benign on a single-core MCU (the ISR cannot fire mid-instruction), but the pattern breaks silently if the firmware ever uses both RP2040 cores |
| 52–63 | MEDIUM | Bad pattern | `static bool init` early-return pattern is fragile and non-obvious; the first call silently skips the full control tick; a dedicated `controlInit()` call (which already exists but does nothing) would be the right place for this |
| 91 | LOW | Magic constant | `127.0f` is the command range upper bound; should be a `constexpr` so it matches the clamp values on lines 98–113 |
| 98–113 | MEDIUM | Code duplication | Eight manual if-comparisons to clamp four motors; a one-liner `std::clamp(c, -127, 127)` eliminates all of this |
| 121–124 | MEDIUM | Incomplete telemetry | Only FL motor state (`tele_tpos_fl`, `tele_pos_fl`, `tele_err_fl`) is updated in the hold path; FR, BL, and BR are invisible when debugging hold failures |

### `pid_velocity.cpp`

| Line | Severity | Category | Issue |
|---|---|---|---|
| 32–33 | LOW | Magic constant | `kF = 255.0f / MAX_TICKS_S` and `kS = 25.0f` are feedforward constants defined inline; they belong in config or as named `constexpr` values |
| 53–57, 78–82 | MEDIUM | Redundant recomputation | `iPWM_now = ki * s.iTerm` (line 53) and `iTermPWM = ki * s.iTerm` (line 78) compute and clamp the same quantity twice; store the clamped result from the anti-windup block and reuse it |
| 125–130 | MEDIUM | Incomplete instrumentation | `u_raw_out` is passed as `nullptr` for FR, BL, BR; raw PWM telemetry only exists for FL making per-motor debugging asymmetric |

### `hold.cpp`

| Line | Severity | Category | Issue |
|---|---|---|---|
| 26 | MEDIUM | Logic | `s.iTerm += err * DT` runs unconditionally on the first tick, before `s.first` is cleared; the derivative term is then zero, but the integrator has already accumulated one sample of error |
| 34–37 | MEDIUM | Missing guard | No `std::isfinite(v_cmd)` check before clamping; if a future config change produces `DT = 0`, `dErr` becomes ±inf and propagates through to the motor command |
| 45 | MEDIUM | Type safety | `abs(err)` resolves to `<cstdlib>` `abs(int)` not `std::abs(int32_t)`; silently truncates on platforms where `int` is 16-bit; use `std::abs(err)` |
| 45 | LOW | Magic constant | Deadband threshold `6` ticks has no label; name it `constexpr int32_t kHoldDeadbandTicks = 6` |

### `encoders.cpp`

| Line | Severity | Category | Issue |
|---|---|---|---|
| 6–24 | MEDIUM | Code duplication | Four ISR functions (`isrFL`–`isrBR`) are identical except for the pin constants and accumulator variable; one macro or inline template eliminates the duplication and prevents copy-paste drift |
| 8–24 | MEDIUM | ISR safety | Two sequential `digitalRead()` calls inside an ISR are not atomic; if the encoder transitions between the two reads, the direction decode is wrong; on a Raspberry Pi Pico, raw GPIO register reads (`sio_hw->gpio_in`) are faster and single-cycle |

### `i2c_iface.cpp`

| Line | Severity | Category | Issue |
|---|---|---|---|
| 53–56, 77–80 | MEDIUM | Silent discard | The byte-drain loop (`while (rem-- > 0) Wire1.read()`) silently drops unexpected extra bytes with no log; a wrong byte count from the host leaves no trace |
| 100–106, 126–132 | MEDIUM | Code duplication | `pack32` lambda defined identically twice (once under `REG_ENCODERS`, once under `REG_TELEM_FL`); extract to a static file-scope helper |
| 141 | MEDIUM | Silent fallback | Unknown `currentReg` values silently send a zero byte; add an `RCLCPP_WARN` (or Serial.print) so bad register accesses are visible during development |

### `shared.hpp`

| Line | Severity | Category | Issue |
|---|---|---|---|
| 26–51 | MEDIUM | Design | Fourteen `volatile` globals with no documented ISR ownership; comments label them as "shared state" but don't specify which ISR or function writes each one; this makes it impossible to audit race conditions without reading every file |
| 49–51 | LOW | Inconsistency | `MotorState` and `HoldState` PID-state objects are declared `extern` alongside the volatile hardware globals but are never accessed from an ISR; they should be separated into a non-interrupt section to clarify that only encoder/command values need `volatile` |

---

## `src/odom_node/`

| File | Line | Severity | Category | Issue |
|---|---|---|---|---|
| `odom_node.cpp` | 32 | HIGH | Missing guard | No protection on `ticks_per_rev` division — see Critical Issues §6 above |
| `odom_node.cpp` | 32 | LOW | Style | Hardcoded `3.141592653589793` instead of `M_PI` from `<cmath>` (already included) |
| `odom_node.cpp` | 83 | LOW | Readability | `if (!(dt > 0.0))` is the double-negation form of `if (dt <= 0.0)`; the latter is unambiguous |
| `odom_node.cpp` | 135–147 | MEDIUM | Magic constants | Covariance values (`base=0.5`, `mag=10.0`, `IGN=1e6`) are hardcoded without being exposed as ROS parameters; can't be tuned at launch time without recompiling |
| `odom_node.cpp` | 141, 149 | MEDIUM | Hot-path waste | Two 36-element covariance arrays are zeroed with range-for loops then overwritten with 6 values on every callback (~20 Hz); pre-initialize both arrays once in the constructor and reuse the message object as a member variable |
| `odom_node.hpp` | 29–35 | MEDIUM | Design | Pose state (`x_`, `y_`, `theta_`) accumulates from node start and is never reset; if the node is restarted, the old pose persists until re-initialized by an external localization source |

---

## `src/mecanum_drive_controller/`

| File | Line | Severity | Category | Issue |
|---|---|---|---|---|
| `mecanum_drive_controller.cpp` | 77–81 | HIGH | Logic bug | Fatal I2C error not fatal — see Critical Issues §2 above |
| `mecanum_drive_controller.hpp` | 46 | HIGH | Uninitialized member | `i2c_file_` has no default — see Critical Issues §3 above |
| `mecanum_drive_controller.cpp` | 119, 128 | MEDIUM | Hot-path allocation | `i2cWriteI8` and `i2cWriteU8` heap-allocate a `std::vector` on every call; these run at 20Hz+ writing 5-byte packets; use a fixed-size stack array (`uint8_t buf[13]`) |
| `mecanum_drive_controller.cpp` | 161–163 | MEDIUM | Logging level | `RCLCPP_WARN` on normal startup output ("Sent PID to Pico: ..."); this is informational, not a warning |
| `mecanum_drive_controller.cpp` | 172 | MEDIUM | Error context | "Encoder read failed" log provides no errno or I2C error details; hard to distinguish bus error from firmware crash |
| `mecanum_drive_controller.cpp` | 244–256 | MEDIUM | Hot-path string format | Throttled `RCLCPP_INFO_THROTTLE` formats 12 floating-point values into a 150+ character string every 1000 ms; when it fires it causes a small latency spike in the control thread; move to DEBUG level or remove for production |
| `mecanum_drive_controller.cpp` | 209 | LOW | Redundant computation | `ticks_per_meter = 1.0 / std::max(distance_per_tick_, 1e-9)` is a constant (same denominator as the constructor calculation) recomputed on every `cmdVelCallback`; precompute and store as a member alongside `distance_per_tick_` |
| `mecanum_drive_controller.cpp` | 227–233 | LOW | Style | `to_i8` manually clamps with `std::max/std::min`; `std::clamp(u, -1.0, 1.0)` is available and clearer in C++17 |
| `mecanum_drive_controller.cpp` | 235–236 | LOW | Redundant computation | `inv_max_ticks = 1.0 / max_ticks_per_sec_` recomputed on every `cmdVelCallback` invocation; precompute once at construction and store as a member |

---

## `src/paesano_localization/`

### `particle_filter.cpp`

| Line | Severity | Category | Issue |
|---|---|---|---|
| 326–336 | HIGH | Memory/logic bug | `free_cells_` not cleared — see Critical Issues §5 above |
| 40 | MEDIUM | Logic | `num_random_max_` is clamped independently of `num_random_`; the post-check `if (num_random_max_ < num_random_) num_random_max_ = num_random_` on line 40 should be unified into a single validated assignment |
| 82–84 | MEDIUM | Dead parameters | Particles initialized around (0, 0, 0) unconditionally; `init_x_`, `init_y_`, `init_yaw_` are declared and loaded from ROS parameters (lines 57–59) but are never added to the initial particle poses |
| 284–291 | MEDIUM | Redundant recomputation | The weighted mean pose (sin/cos of each particle's theta × weight) is computed once in `scanCallback` for publishing, then computed again in `broadCastMapToOdomTf` (pf_tf_helpers.cpp:65–69) for the TF broadcast; 250 particles × 2 trig ops × 10 Hz = 5,000 wasted trig calls per second; compute once and pass through |

### `pf_compute_score.cpp`

| Line | Severity | Category | Issue |
|---|---|---|---|
| 203 | HIGH | Hot-path allocation | `std::vector<double> logw(particles_.size())` allocated fresh on every scan — see Critical Issues §8 above |
| 240–241 | HIGH | Expensive op in loop | 3 transcendentals (2× `exp`, 1× `log`) per beam per particle in the scoring inner loop — see Critical Issues §9 above |
| 101–108 | HIGH | Hot-path allocation | ~16 MB of temporary vectors allocated on every distance field rebuild — see Critical Issues §10 above |
| 194–248 | MEDIUM | Redundant computation | Beam validity is checked twice per scan: once in a pre-pass to count valid beams (lines 194–200), and again inside the particle loop for every particle (lines 215–221); build a valid-beam index list once outside the particle loop |
| 181–187 | MEDIUM | Redundant computation | `log_norm`, `log_z_hit`, `log_z_rand`, `log_rand` are recomputed inside `score()` on every scan; these depend only on `sigma_hit_`, `z_hit_`, and `z_rand_` which never change at runtime; cache them as member variables computed once when parameters are set |

### `pf_random_helpers.cpp`

| Line | Severity | Category | Issue |
|---|---|---|---|
| 12, 20 | MEDIUM | Hot-path allocation | `std::uniform_real_distribution` and `std::normal_distribution` objects are constructed fresh on every call; `score()` calls these thousands of times per scan callback; cache the distributions as members or function-static objects seeded once at construction |

### `covariances_on_imu.cpp`

| Line | Severity | Category | Issue |
|---|---|---|---|
| 26–29 | MEDIUM | C++ style | `fill_cov()` takes a raw `float*` pointer into a `std::array`; use `std::span<float, 9>` (C++20) or an explicit `std::array<float, 9>&` to preserve type safety and size information |
| 41–50 | MEDIUM | Hardcoded tuning | IMU noise-model constants (`VAR_ACC_LO`, `VAR_ACC_HI`, etc.) are `constexpr` in the .cpp file; they should be node parameters so the filter can be tuned per-sensor without recompiling |

### `load_map.cpp`

| Line | Severity | Category | Issue |
|---|---|---|---|
| 61–63 | MEDIUM | Resource waste | A temporary `rclcpp::Node`, `ServiceClient`, and `SingleThreadedExecutor` are created on every `loadMap()` call; all three should be class members initialized once in the constructor |
| 61–63 | MEDIUM | Potential deadlock | `helper_executor.spin_until_future_complete()` blocks the calling ROS2 executor for the full map-server round-trip; if `loadMap()` is ever invoked from inside a callback on the same executor that the map server uses, the response can never be delivered and the call hangs forever |
| 66, 79 | LOW | Magic constants | Service wait timeout (2 s) and response timeout (10 s) are hardcoded; expose as ROS parameters |

---

## `src/paesano_navigation/`

### `a_star_action_server.cpp`

| Line | Severity | Category | Issue |
|---|---|---|---|
| 98–100 | HIGH | Threading | Detached thread captures `this` — see Critical Issues §4 above |
| 98–100 | HIGH | Data race | `execute()` thread reads `current_pose_`, `have_map_`, `have_pose_`, and `planner_` without synchronization while executor thread writes them — see Critical Issues §7 above |

### `a_star.cpp`

| Line | Severity | Category | Issue |
|---|---|---|---|
| 81–89 | HIGH | Hot-path allocation | ~28 MB of vectors allocated per `plan()` call — see Critical Issues §11 above |
| 78, 103–110, 153 | MEDIUM | Memory bloat | Lazy-deletion open set: when a cell is relaxed, a new entry is pushed without removing the old one; on large maps the queue can hold 2–5× more entries than necessary, causing extra heap allocations and wasted pop+comparison cycles; use a visited map to skip stale entries earlier, or track open-set membership separately |
| 151 | MEDIUM | Redundant recomputation | `heuristic(neighbor, goal_cell)` is called every time a cell's g-score is improved; the heuristic is deterministic (depends only on neighbor position and fixed goal), so the same value is recomputed repeatedly; precompute all `h_score` values into a flat array at the start of `plan()` with a single pass over the grid |
| 30 | MEDIUM | Code duplication | `InternalPoint` struct is defined in both `a_star.cpp` and `a_star_helpers.cpp`; the two definitions must be kept in sync manually; move to the shared header |
| 84 | MEDIUM | Type safety | `std::vector<int> came_from` uses signed `int` for grid indices; large maps (>2 million cells) overflow; use `std::vector<int32_t>` or `size_t` consistently |

### `a_star_helpers.cpp`

| Line | Severity | Category | Issue |
|---|---|---|---|
| 193–195, 226 | HIGH | Redundant recomputation | Catmull-Rom parameterisation recomputed 100× per segment — see Critical Issues §12 above |
| 86–116 | MEDIUM | Algorithm complexity | `stringPull()` path simplification is O(n²) — already noted; quick win: reverse the search direction and break on first success to reduce to expected O(n) |
| 214–244 | MEDIUM | Hot-path allocation | `dense_points` vector in `applySpline()` grows via repeated `push_back()` without a prior `reserve()`; for a 10-segment path at 100 samples per segment, this triggers multiple reallocations; add `dense_points.reserve(path.poses.size() * 100)` |
| 341–390 | MEDIUM | Redundant computation | `getNeighbors()` calls `isCellTraversable()` on each cardinal direction, then calls it again redundantly when checking diagonals (e.g., `isCellTraversable(left)` is checked for the left neighbor and again as part of the top-left diagonal guard); cache the four cardinal results and reuse them for the diagonal guards |
| 251–254 | MEDIUM | Algorithm complexity | The inner `while` loop in `applySpline()` that advances `dense_idx` is a linear scan; for long paths with many dense samples it degrades to O(n) per output point; replace with `std::lower_bound` on the pre-sorted `dense_points` distance field |
| 60–62 | LOW | Redundant computation | `isCellTraversable()` rechecks that `map_inflated_.data.size() == map_.data.size()` on every call; this runs ~50,000 times per large A* search; move the check to a debug assertion or validate once in `setMap()` |
| 341 | LOW | Unnecessary copy | `getNeighbors()` returns `std::vector<Coordinate>` by value; pass the output vector by reference and `clear()` it at the start to eliminate the move overhead in the range-for at a_star.cpp:129 |
| 32 | LOW | Missing `const` | `OpenSetCompare::operator()` is not marked `const`; required for use in `std::priority_queue` with const-qualified comparators |

### `a_star.hpp`

| Line | Severity | Category | Issue |
|---|---|---|---|
| 9 | LOW | Missing virtual dtor | `AStarPlanner` is a public class with no virtual destructor; add one even if subclassing isn't planned today |

---

## `src/paesano_traj_following/`

### `lqr_helpers.cpp`

| Line | Severity | Category | Issue |
|---|---|---|---|
| 18 | MEDIUM | Magic constant | DARE solver iteration cap is `1000` hardcoded; should be a named constant or constructor parameter |
| 21 | MEDIUM | Magic constant | Convergence tolerance `1e-6` is hardcoded; make it a named constant or parameter alongside the iteration cap |
| 34–35 | MEDIUM | Redundant computation | `std::cos(current_pose.theta)` and `std::sin(current_pose.theta)` are computed in `calculateError()` here; the same angle is used again in `lqr.cpp:144–145` for the feedforward term in the same control tick; cache the sin/cos pair as a member or pass them through to avoid the second trig evaluation |
| 66 | MEDIUM | Expensive op in loop | `std::pow(path[i].x - robot_pose.x, 2)` in the 50-point closest-index search; `std::pow(x, 2)` is significantly slower than `dx * dx`; this runs on every control tick at 50 Hz |

### `lqr.cpp`

| Line | Severity | Category | Issue |
|---|---|---|---|
| 144–145 | MEDIUM | Redundant computation | `sin` and `cos` of `current_pose.theta` recomputed here for the feedforward term; `calculateError()` (called 3 lines earlier at line 141) already computed the same values; cache them |
| 150 | MEDIUM | Magic constant | `0.5 * wrapAngle(...)` feedforward gain is hardcoded; the other LQR gains (Q, R matrices) are declared as parameters — this one should be too |
| 60–63 | LOW | Design note | Path search window of 50 points is documented as an RPi 5 performance tradeoff (comment on line 60); acceptable, but the silent failure mode is that the robot cannot re-acquire the path after a large deviation without triggering a replanning action |
