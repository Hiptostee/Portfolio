# Paesano Things to Fix

Major issues can break or misrepresent autonomous behavior. Medium issues are needed for a
dependable demo. Minor issues are cleanup and better visibility.

## Major

### Separate localization from learned map changes

Do not write local obstacle observations directly into the saved localization map. Keep the
localization map stable and learn repeated environmental changes in a separate persistent layer.
The planning map may conservatively combine static, persistent, and live occupied cells.

Promote a new obstacle only after repeated observations across time or viewpoints. Never erase a
static obstacle because one local scan reports free space; require repeated clear rays, localization
confidence, and an explicit versioned map update or human confirmation. The particle filter
currently globally reinitializes whenever it receives a new map, so live map mutation would also
disrupt localization.

The current planning-map merge ignores free cells from `/local_map`, so furniture captured in the
saved map can never be cleared for planning even after it moves. Add per-cell occupied and clear
evidence plus a high-confidence clearing mask for `/planning_map`. Count at most one vote per cell
per scan and gate accepted votes by elapsed time or changed viewpoint; three adjacent 10 Hz scans
are correlated, not three independent confirmations. Publish or reinflate the global planning map
only when cell state changes or at a bounded rate because A* currently rebuilds its full inflated
map on every planning-map message.

### Finish the frontier algorithms

Implement frontier detection, eight-connected clustering, and safe-goal selection with scoring,
standoff, and failed-goal filtering.

### Build everything in ROS 2 Jazzy

Build the mapping, orchestrator, explorer, description, and bringup packages with `colcon`.
Confirm that every component plugin is discoverable and each launch mode starts without duplicate
nodes or TF publishers.

### Check the complete mapping flow

Confirm there is one `map -> odom` publisher, the mapping pose component publishes
`/estimated_pose`, A* receives `/planning_map`, LQR receives `/path`, and navigation results return
to the explorer.

```text
/map -> frontier goal -> orchestrator -> /a_star -> /path -> LQR -> /navigation/result
```

### Stop retrying permanent obstacles forever

Give replanning a maximum attempt count or timeout. If the route stays blocked, return a failure
and let the explorer try another frontier.

### Report when LQR fails

If LQR stops because tracking error is too high, the orchestrator should publish
`TRACKING_FAILED` instead of staying in `NAVIGATING` forever.

### Add a navigation timeout

If a goal never succeeds or fails normally, stop safely and publish `TIMED_OUT`.

### Keep one navigation-goal owner

Do not let a mobile, manual, semantic, or exploration goal silently replace another active goal.
Switching modes should explicitly cancel or finish the current request.

### Make failed-frontier filtering spatial

Reject goals within `blacklist_radius_m` of a failed goal, not only the exact coordinate. Make sure
small SLAM shifts do not cause Paesano to retry neighboring points from the same unreachable
frontier forever.

For a large frontier, allow only a bounded number of alternative approach points before treating
that region as unreachable.

## Medium

### Separate complete from stuck

No frontiers and only tiny frontiers can mean the map is complete. Frontiers that still exist but
are all blacklisted or have no safe goal mean exploration is partial or stuck, not fully complete.

### Add pause and cancel

Pausing exploration should stop motion safely. Resuming should continue selecting frontiers, and
canceling should return the robot to idle.

### Save the map when exploration finishes

After confirmed completion, save the occupancy map and report where it was stored. Decide later
whether the `slam_toolbox` pose graph should also be saved.

### Share runtime launch logic

`paesano_bringup.launch.py` launches the hardware runtime and
`paesano_description.launch.py` launches the Gazebo runtime. They currently duplicate the mapping,
localization, navigation, orchestrator, and `auto_explore` conditions.

After autonomous exploration works, extract the shared runtime composition so simulation and
hardware cannot drift apart.

## Minor

### Improve exploration diagnostics

Log the raw frontier count, retained cluster count, blacklisted count, selectable-goal count, and
the reason each candidate was rejected.

### Confirm RViz exploration markers are useful

Check that frontier cells and the selected goal are easy to distinguish. Adjust marker size,
color, or lifetime only if the current visualization is hard to debug.

### Keep exploration documentation synchronized

After interfaces or completion behavior change, update the explorer, mapping, description, and
bringup READMEs along with `AGENTS.md`.

## Already Fixed

### Empty startup maps no longer look complete

The explorer must issue at least one frontier goal before `COMPLETE` is allowed.

### Simulation and hardware expose the same exploration switch

Both launch paths now accept `auto_explore:=true`. They remain duplicated until the shared-runtime
cleanup above.
