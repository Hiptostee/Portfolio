## Paesano Experiments Plan

### Summary
Run all three experiments on real hardware, record each trial with `ros2 bag`, and process the bags in Python into one figure/table per subsection. Use taped floor marks as ground truth. Do not add new sensing hardware. Keep the experiment scope minimal: 3 motion conditions for control, 5 marked poses for localization, and 3 route types for trajectory following.

### Implementation Plan

#### 1. Common setup
- Use the real hardware stack in `localization` mode for the localization and trajectory experiments, and a minimal/manual mode for the velocity-control experiment.
- Mark a clean test area on the floor with tape. Make 5 fixed pose markers for localization and 3 repeatable route layouts for trajectory following.
- Use one saved hardware map for all trials so every measurement is in the same `map` frame.
- Record every trial with `ros2 bag record` into a clearly named folder: experiment name, route/condition, trial number, timestamp.
- Create one Python analysis script or notebook that reads the bags and outputs plots/tables for the report.

#### 2. Velocity and Position Control Performance
- Run this as a low-level control experiment, separate from path following.
- Use a scripted `cmd_vel` profile instead of joystick teleop so the inputs are repeatable.
- Test 4 conditions:
  - forward step: `v_x`
  - lateral step: `v_y`
  - angular step: `omega`
  - stop-and-hold after commanded motion
- For each condition, run 3 trials.
- Record:
  - `/cmd_vel`
  - `/odom`
  - `/odom/filtered`
  - optionally `/wheel_encoders` if you want one deeper debug signal
- Primary metrics:
  - rise time
  - settling time
  - steady-state velocity error
  - overshoot
  - stop drift over 3-5 seconds after zero command
- Use `/odom.twist.twist` as the main measured-velocity source, with `/odom/filtered` as a sanity check rather than the primary metric.
- Report output:
  - one plot of commanded vs measured velocity for each axis tested
  - one small table summarizing the metrics above

#### 3. Localization Performance
- Use 5 taped floor poses distributed across the mapped area: center, near two walls, and two corners/open-area transitions.
- For each floor pose, mark both position and heading on the floor so placement is repeatable.
- Establish the map-frame ground-truth coordinates once by careful placement and averaging at each marker, then use those fixed coordinates for repeated trials.
- At each marker:
  - place the robot carefully on the taped pose
  - let localization settle for 3-5 seconds
  - record 5 seconds of pose data
  - repeat 3 times per marker
- Record:
  - `/estimated_pose`
  - `/odom/filtered`
  - `/map`
  - optionally `/particle_cloud` if you want one visual figure
- Primary metrics:
  - mean position error at each marker
  - max position error
  - mean heading error
  - standard deviation across repeated trials
- Compute the pose estimate for each trial as the average over the stable 5-second window rather than a single sample.
- Report output:
  - one table of errors by marker
  - one figure showing the test markers on the map

#### 4. Trajectory Following Performance
- Use 3 route types:
  - straight-line route
  - right-angle route
  - multi-segment route through a cluttered area
- Run each route 3 times in localization mode on the same saved map.
- Use the same start pose for all repeats of a route, marked physically on the floor.
- Send goals consistently; if you want heading included, keep the final heading fixed and known for each route. If not, evaluate position and path-tracking error only.
- Record:
  - `/path`
  - `/estimated_pose`
  - `/odom/filtered`
  - `/cmd_vel`
  - `/is_navigating`
  - `/map`
- Primary metrics:
  - final position error
  - cross-track error along the route
  - completion time
  - success/failure rate
- Optional secondary metric:
  - final heading error, only if you deliberately command a meaningful terminal heading for the route
- Compute cross-track error by comparing the estimated robot pose samples against the planned path geometry from `/path`.
- Report output:
  - one overlay figure of planned path vs actual path for each route type
  - one summary table with final error, mean cross-track error, and completion time

### Test and Analysis Workflow
- First run one pilot bag for each experiment before collecting the full dataset.
- Confirm the pilot bags contain the expected topics and timestamps before doing repeated trials.
- After the pilot passes, collect the full dataset in one session per experiment to keep conditions consistent.
- Analysis script outputs:
  - PNG/SVG plots for the paper
  - CSV summary tables for easy LaTeX insertion
- Mark the experiments done only when each subsection has:
  - at least 3 repeated trials per condition
  - one figure
  - one table
  - one paragraph of interpretation

### Assumptions and Defaults
- Real hardware only; no additional camera-based ground-truth system.
- Ground truth is tape marks plus manually established map-frame reference poses.
- Data collection uses `ros2 bag`; analysis uses Python.
- Velocity-control experiment uses scripted `cmd_vel`, not the mobile app, to avoid input-timing noise.
- Localization and trajectory experiments use the saved hardware map in localization mode.
- Final heading is optional for trajectory evaluation unless you deliberately make it part of the commanded route objective.
