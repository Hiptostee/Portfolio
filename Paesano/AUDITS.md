# Paesano Engineering Audits

Living checklist for known runtime risks, incomplete failure handling, and verification debt.
Update an item only after its completion condition has been demonstrated.

## Autonomous Exploration

### AE-001 — Implement frontier algorithms

- [ ] Implement frontier detection in `paesano_explorer`.
- [ ] Implement eight-connected frontier clustering.
- [ ] Implement safe-goal selection, scoring, standoff, and failed-goal filtering.

Risk: the explorer currently produces no navigation goals because the algorithms are stubs.

Done when: a synthetic or simulated unknown map produces visible frontier clusters and one
known-free, reachable navigation goal.

### AE-002 — Bound permanent-obstacle replanning

- [ ] Track consecutive replan failures for the active navigation goal.
- [ ] Add a configurable maximum attempt count or navigation timeout.
- [ ] Publish a terminal navigation failure after the limit is reached.

Risk: a permanent obstruction can leave the orchestrator retrying the same destination forever.

Done when: a permanently blocked path ends with a failure result, and the explorer selects a
different frontier.

### AE-003 — Report controller tracking failures

- [ ] Give the orchestrator a reliable indication that LQR stopped before reaching the goal.
- [ ] Publish `TRACKING_FAILED` through `/navigation/result`.
- [ ] Make the explorer blacklist the failed destination and continue.

Risk: LQR can stop after excessive tracking error while the orchestrator remains in
`NAVIGATING` indefinitely.

Done when: an induced tracking-error stop produces `TRACKING_FAILED` instead of success or a
permanent `NAVIGATING` state.

### AE-004 — Add navigation timeout

- [ ] Record the start time of each navigation request.
- [ ] Add a configurable goal timeout.
- [ ] Stop LQR and publish `TIMED_OUT` when the deadline is exceeded.

Risk: a goal that neither succeeds nor produces a recognized failure can remain active forever.

Done when: an intentionally stalled goal stops safely and produces `TIMED_OUT` at the configured
deadline.

### AE-005 — Enforce one navigation-goal owner

- [ ] Prevent mobile, manual, or semantic goals from silently replacing an exploration goal.
- [ ] Define how switching away from autonomous exploration cancels the active goal.
- [ ] Associate navigation results with the correct request before adding multiple goal sources.

Risk: `/navigation/result` currently assumes one active goal owner and contains no goal ID.

Done when: a manual command during exploration is rejected, queued, or explicitly cancels
exploration without misattributing the result.

### AE-006 — Add exploration pause and cancel

- [ ] Add pause, resume, and cancel control interfaces.
- [ ] Stop LQR when exploration is paused or canceled.
- [ ] Preserve or intentionally clear the failed-goal list on resume.

Risk: exploration currently runs until completion or node shutdown.

Done when: pause stops motion, resume continues frontier selection, and cancel returns the system
to a safe idle state.

### AE-007 — Save the completed map automatically

- [ ] Trigger map saving only after confirmed `COMPLETE`.
- [ ] Save the occupancy map YAML/image.
- [ ] Decide whether to serialize the `slam_toolbox` pose graph for resumed mapping.
- [ ] Publish or log the saved map path and failure status.

Risk: exploration can report completion without preserving the resulting map.

Done when: a completed simulated exploration creates a loadable map without a manual command.

## Verification Debt

### AV-001 — Build the new component graph in ROS 2 Jazzy

- [ ] Build `paesano_mapping`, `paesano_orchestrator`, `paesano_explorer`, and bringup with `colcon`.
- [ ] Confirm all component plugins are discoverable.
- [ ] Launch each bringup mode and check for duplicate nodes, topics, or TF publishers.

Reason: the current host environment does not have `colcon`, so only static syntax, package XML,
dependency, and interface checks have been completed.

Done when: all affected packages build and the three launch-mode combinations start without ROS
errors.

### AV-002 — Validate mapping-mode topic and TF flow

- [ ] Confirm exactly one `map -> odom` publisher during mapping.
- [ ] Confirm `/estimated_pose` is published by `MappingPosePublisher` during exploration.
- [ ] Confirm A* receives `/planning_map` and `/estimated_pose`.
- [ ] Confirm `/navigation/result` returns to the explorer after success and planning failure.

Done when: the full chain works in simulation:

```text
/map -> frontier goal -> orchestrator -> /a_star -> /path -> LQR -> /navigation/result
```

## Resolved Findings

### AR-001 — Premature exploration completion

- [x] Require at least one issued frontier goal before allowing the explorer to enter `COMPLETE`.

Resolution: `exploration_started_` now prevents an empty startup map from being mistaken for a
finished exploration.
