# Paesano Agent Guide

This file defines how coding agents should work in the Paesano repository.

## Start Here

Read the documents relevant to the task before editing:

1. [`CodeStandard.md`](CodeStandard.md) — mandatory code, package, interface, launch, and README conventions.
2. [`AUDITS.md`](AUDITS.md) — known risks, incomplete failure handling, verification debt, and acceptance conditions.
3. [`plan.md`](plan.md) — active Minimal Alive Paesano roadmap and current implementation order.
4. [`long_term_plan.md`](long_term_plan.md) — long-term embodied cognition, memory, personality, and safe self-improvement vision.

Do not mark a plan or audit item complete merely because code was written. Satisfy its stated
acceptance condition or describe what remains unverified.

## New Rules from Joseph

When Joseph gives a new durable rule about this repository, architecture, workflow, safety,
documentation, or how agents should operate, update `AGENTS.md` during the same task so future
agents receive it too.

Keep the rule concise and place it in the most relevant section. Do not record secrets, private
credentials, or clearly temporary one-command instructions in `AGENTS.md`.

When work reveals a known issue that will remain unfixed, add it to `AUDITS.md`:

- Major: can break, deadlock, or misrepresent autonomous behavior.
- Medium: needed for a dependable demonstration or maintainable workflow.
- Minor: cleanup, diagnostics, visualization, or documentation quality.

Keep audit entries personal and simple. Use descriptive names and short explanations, not IDs,
formal risk templates, or checkbox lists.

## Project Context

Paesano is a custom indoor mecanum-drive robot built on ROS 2 Jazzy. The repository contains the
robot description, sensor drivers, odometry and localization, mapping, planning, trajectory
following, dynamic-obstacle handling, orchestration, mobile control, and autonomous-exploration
work.

Important packages include:

- `paesano_mapping`: `slam_toolbox` launch/config and mapping-mode pose publication.
- `paesano_localization`: EKF support and custom particle-filter localization against a saved map.
- `paesano_navigation`: custom A* planner exposed through `/a_star`.
- `paesano_traj_following`: LQR trajectory follower that consumes `/path`.
- `paesano_local_map`: live robot-centered obstacle layer.
- `paesano_orchestrator`: owns navigation goals, planning, obstacle waiting/replanning, and results.
- `paesano_explorer`: frontier detection, clustering, selection, and exploration state.
- `paesano_semantic_mapping`: converts the occupancy grid into clean structural floorplans and
  stable room, hallway, and doorway regions.
- `paesano_bringup`: top-level mode and hardware launch coordination.

The current active build order is autonomous mapping first, then an adaptive planning-map MVP,
followed by spatial regions, room/hallway classification, RGB-D perception, object-grounded room
classification, persistent memory, voice, and safe LLM actions.

The active near-term schedule is:

- Wednesday, July 22, 2026: autonomous mapping MVP.
- Immediately after autonomous mapping: adaptive planning-map MVP for moved dorm-room obstacles.
- Wednesday, July 29, 2026: room, hallway, and doorway segmentation with stable region IDs.
- Wednesday, August 5, 2026: periodic stopped RGB-D scans projected into the map, followed by
  post-mapping region assignment and room classification.
- Thursday, August 6, 2026 onward: begin the separate local-first memory SDK.

Treat these as minimum working-system milestones. Do not let optional polish expand the current
week; move remaining hardening to `AUDITS.md`. The dated definitions of done live in `plan.md`.
If adaptive planning-map work conflicts with room segmentation, finish the adaptive-map MVP first;
database persistence and versioned map consolidation remain later hardening.

## Navigation Ownership

Preserve these responsibility boundaries:

```text
Goal source
  -> /navigation/goal
  -> orchestrator
  -> /a_star action
  -> /path
  -> LQR
  -> /cmd_vel
  -> motor controller
```

- Goal sources such as the explorer, mobile bridge, semantic navigation, or an LLM do not call
  A* or publish `/cmd_vel` directly.
- A* computes a path; it does not own the complete navigation lifecycle.
- The orchestrator is the single owner of active navigation state and recovery policy.
- For a dynamically blocked path, preserve the current goal during the short wait and bounded
  replan window. Defaults are a 5-second wait, replans every 2 seconds, and failure after a
  20-second recovery window; keep all three timings configurable in the orchestrator.
- LQR and the motor-control path own velocity commands.
- The intelligence layer may propose structured intentions but must never bypass deterministic
  planning, collision stopping, command validation, or emergency-stop behavior.

## Mapping and Localization Modes

The top-level launch supports these important combinations:

```text
localization_mode:=false auto_explore:=false
  Manual mapping with slam_toolbox.

localization_mode:=false auto_explore:=true
  SLAM, mapping pose publication, A*, LQR, orchestrator, and frontier explorer.

localization_mode:=true auto_explore:=false
  Saved-map particle-filter localization, A*, LQR, and orchestrator.
```

Use `paesano_bringup.launch.py` for hardware and `paesano_description.launch.py` for Gazebo. Both
currently compose the runtime stack independently, so keep their mode and `auto_explore`
conditions synchronized when changing launch behavior.

During mapping, `slam_toolbox` owns `map -> odom`. During saved-map localization, the particle
filter owns `map -> odom`. Never launch two publishers for that transform.

`/estimated_pose` also has one mode-dependent source:

- Mapping exploration: `paesano_mapping::MappingPosePublisher` converts `map -> base_link` TF to
  `/estimated_pose`.
- Saved-map localization: the particle filter publishes `/estimated_pose`.

`/map` is the one persistent world occupancy map. `/local_map` is a transient obstacle layer.
The orchestrator overlays them into `/planning_map` for A*; it does not create a second SLAM map.

After initial SLAM, derive a separate clean structural floorplan containing room and hallway
regions, door connections, stable region IDs, and simplified boundaries in the `map` frame. Use
this semantic floorplan for the mobile dashboard, room memory, and human-facing visualization.
Do not feed simplified or inferred geometry back into the particle filter or treat it as the
collision-planning map; localization continues using the original saved occupancy map, while A*
uses `/planning_map`.

Treat the saved `/map` used by the particle filter as a stable localization reference during a
run. Do not let a local occupied or free observation rewrite it directly. Future long-lived
environmental changes belong in a separate, versioned persistent-change layer: repeated occupied
evidence may conservatively block planning, while clearing a static obstacle requires stronger
multi-view evidence or human confirmation. `/planning_map` may combine these layers without
changing the localization reference.

Run persistent change mapping alongside localization during normal operation, not only during
initial SLAM. Confirmed learned changes must survive restarts, remain tied to the exact reference
map version or identity that produced them, and stay reversible without mutating that reference
map. Save learned state atomically at a bounded interval rather than writing on every scan.

When implementing persistent changes, use per-cell occupied and clear evidence from LiDAR rays.
Aggregate no more than one vote per cell per scan and gate confirmations by time or viewpoint;
consecutive 10 Hz readings are correlated. A validated clear mask may override stale occupied
furniture in `/planning_map`, but not in the localization reference. Publish planning-map changes
at a bounded rate or only when a cell changes state because A* rebuilds global inflation on every
received planning map.

## Autonomous Exploration Status

Frontier detection, eight-connected clustering, initial scoring, spatial failed-goal filtering,
and the first selector implementation exist. They are not yet accepted as working autonomous
exploration. A simulation rosbag showed two large valid clusters, but both selected goals remained
only one cell from unknown space and were rejected by A*'s inflated map.

Use these rules when finishing goal selection:

- Compute the approach direction away from nearby unknown cells. Do not assume that the vector
  toward the robot points inward from the frontier; it can run tangent to an irregular boundary.
- Validate exploration candidates against the latest geometry-compatible `/map_inflated`, because
  raw-map free space can still be non-traversable under A*'s obstacle buffer.
- Try only a bounded number of spatially separated approach points from each cluster before treating
  that frontier region as unavailable.
- Declare `COMPLETE` only after repeated map updates contain no retained frontier clusters. If
  clusters remain but every candidate is blocked, unsafe, or blacklisted, report recoverable
  `STUCK` and reconsider the frontiers when relevant map or inflated-map data changes.

Keep algorithm TODOs in `.cpp` files, not public headers. Do not silently fill exploration
implementations unless the user asks for implementation help.

Before expanding exploration behavior, review the `AE-*` and `AV-*` entries in `AUDITS.md`.
In particular, preserve the known single-goal assumption until goal ownership and result IDs are
designed explicitly.

## RGB-D Active Vision

- Use one Intel RealSense D435 as the RGB-D observation camera; do not introduce a second camera
  pipeline unless Joseph explicitly changes this architecture.
- Bench-test camera streams, YOLO, and depth projection before the final CAD mount exists. A
  secured temporary fixed support is sufficient for initial software work.
- First complete the fixed-camera RGB-D pipeline, then mount the camera on a rigid elevated holder
  with one servo-controlled tilt axis. Paesano rotates its chassis for horizontal viewing.
- Represent the tilt joint in TF as `base_link -> camera_tilt_link -> camera_link` so mapped
  observations use the current camera orientation.
- Prefer repeatable down, forward, and up poses. Stop the robot, wait for the tilt mechanism and
  camera exposure to settle, and then capture any observation that will be projected or remembered.
- During mapping, trigger semantic scans after a configurable travel distance, initially about
  1.5-2.0 m, or when the explorer detects entry into a new open space. Coordinate the pause through
  the navigation owner, preserve the active exploration goal, and resume only after the camera has
  returned to its navigation pose.
- A mapping-time semantic scan stops the chassis, tilts the camera upward about 20-35 degrees,
  captures several settled RGB-D frames, runs YOLO and depth projection, transforms valid detections
  into `map`, and stores class, confidence, global position, and timestamp.
- Do not infer room-purpose labels during mapping. After the occupancy map is complete, segment its
  rooms and hallways, assign stored detections to stable regions, aggregate their evidence, and then
  infer room labels.

## Code and Package Rules

Follow `CodeStandard.md`. In particular:

- Custom runtime nodes must be composable ROS 2 components constructed with `rclcpp::NodeOptions`.
- Public headers contain declarations, public types, and member variables only.
- Implementations and file-local helpers belong in `.cpp` files.
- Group semantic-mapping algorithm stages into focused header/source pairs. Keep the semantic
  mapping node source limited to ROS I/O and readable top-level pipeline calls.
- Declare topics, services, actions, frames, and runtime tunables as parameters with code defaults.
- Put normal parameter overrides in package YAML files.
- Keep launch files focused on composition, config loading, and high-level launch arguments.
- Keep package READMEs short: purpose, runtime entry point, config, interfaces, and one command.
- Preserve existing user changes and inspect `git status` before editing.
- Avoid unrelated cleanup while implementing a scoped feature.
- Never add direct motor commands to cognition, exploration, semantic mapping, or memory packages.

Use `apply_patch` for manual file edits. Prefer `rg` and `rg --files` for repository searches.

## Validation

Validate in proportion to the change:

- Run `git diff --check` for edited files.
- Parse modified package XML and launch files.
- Build affected packages in a sourced ROS 2 Jazzy environment with `colcon` when available.
- Confirm component plugins are discoverable after component or CMake changes.
- Check topic types, QoS, frame ownership, and producer/consumer pairs after interface changes.
- Use simulation and rosbag replay before physical hardware for navigation, localization, or safety
  changes.
- Record missing build or hardware verification in `AUDITS.md`; do not imply it passed.

The current host environment may not have `colcon`. Static checks are useful but do not replace a
ROS build or launch validation.

## Plans and Scope

`plan.md` is the active execution checklist. Favor the smallest implementation that satisfies the
current acceptance test. Do not pull post-Christmas cognition, personality, self-improvement, or
large perception work forward unless the user explicitly changes priorities.

`long_term_plan.md` supplies architectural direction, especially these invariants:

- The LLM never controls motors directly.
- Memories retain confidence, provenance, timestamps, and correction history.
- Observations, user statements, model inferences, and verified robot state remain distinguishable.
- Self-improvement is sandboxed, regression-tested, human-approved, and recoverable.

## Career Guidance

When a request concerns Joseph's career, resumes, internship strategy, project positioning,
recruiter communication, or prioritization between career paths, use the `career-supervisor` skill
and read its current profile before advising.

Do not invoke career framing merely to implement routine code. When career guidance is relevant:

- Keep claims measurable and interview-defensible.
- Separate completed hardware evidence from planned or simulated functionality.
- Preserve the verified Paesano narrative: custom localization and controls, simulation/rosbag
  validation, system integration, and measured hardware navigation results.
- Do not claim SLAM, perception, autonomous exploration, room classification, memory, or LLM
  capabilities until their implementation and validation support the claim.

The skill is available as `$career-supervisor`; its source is normally located at
`/Users/josephmarra/.codex/skills/career-supervisor/SKILL.md`.
