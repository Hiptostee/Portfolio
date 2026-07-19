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
- `paesano_bringup`: top-level mode and hardware launch coordination.

The current active build order is autonomous mapping first, followed by spatial regions,
room/hallway classification, RGB-D perception, object-grounded room classification, persistent
memory, voice, and safe LLM actions.

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

During mapping, `slam_toolbox` owns `map -> odom`. During saved-map localization, the particle
filter owns `map -> odom`. Never launch two publishers for that transform.

`/estimated_pose` also has one mode-dependent source:

- Mapping exploration: `paesano_mapping::MappingPosePublisher` converts `map -> base_link` TF to
  `/estimated_pose`.
- Saved-map localization: the particle filter publishes `/estimated_pose`.

`/map` is the one persistent world occupancy map. `/local_map` is a transient obstacle layer.
The orchestrator overlays them into `/planning_map` for A*; it does not create a second SLAM map.

## Autonomous Exploration Status

The exploration ROS flow is scaffolded, but the following algorithms intentionally remain for
Joseph to implement:

- `FrontierDetector::detect`
- `FrontierClusterer::cluster`
- `FrontierSelector::select`

Keep algorithm TODOs in `.cpp` files, not public headers. Do not silently fill these implementations
unless the user asks for implementation help.

Before expanding exploration behavior, review the `AE-*` and `AV-*` entries in `AUDITS.md`.
In particular, preserve the known single-goal assumption until goal ownership and result IDs are
designed explicitly.

## Code and Package Rules

Follow `CodeStandard.md`. In particular:

- Custom runtime nodes must be composable ROS 2 components constructed with `rclcpp::NodeOptions`.
- Public headers contain declarations, public types, and member variables only.
- Implementations and file-local helpers belong in `.cpp` files.
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
