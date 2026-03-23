# A* Planner Build Guide

This file is meant to tell you what you are actually building, where each part should live, and what order to implement it in.

Right now your codebase has two different jobs:

- `a_star_action_server.cpp`
  This is the ROS 2 wrapper. Its job is to talk ROS:
  - receive an action goal
  - receive the `/map`
  - hand the work to the planner
  - return a `nav_msgs/Path`

- `a_star.cpp`
  This is the actual planner logic. Its job is to do the math and search:
  - store the map
  - convert world coordinates into grid cells
  - run A*
  - convert the cell path back into ROS poses

That means:

- the action server should stay thin
- the planner should do the real work
- ROS inputs and outputs should stay in meters
- the search itself should happen in integer grid cells

## Big Picture Flow

This is the full data flow you are building:

1. Another node sends an action goal with `start` and `goal` poses.
2. `a_star_action_server.cpp` receives that goal.
3. The action server checks that a map has already been received on `/map`.
4. The action server calls `planner_.setMap(current_map_)`.
5. The action server calls `planner_.plan(goal->start, goal->goal)`.
6. Inside `a_star.cpp`, the planner converts `start` and `goal` from meters into map cells.
7. Inside `a_star.cpp`, A* runs on those cells.
8. The planner converts the final cell path back into a ROS `nav_msgs/Path`.
9. The action server returns that path in the action result.

So the main thing you are building next is not "more ROS". It is the inside of `AStarPlanner`.

## What The Planner Should Know

Your planner should know:

- the latest `OccupancyGrid`
- the map width and height
- the map resolution
- the map origin
- which cells are free and which are blocked

Your planner should not care about:

- action callbacks
- subscriptions
- ROS executors
- goal handles

That is why `a_star_action_server.cpp` should stay simple and `a_star.cpp` should get more functions.

## Why You Need World/Grid Conversion

Your action goal uses `PoseStamped`, which means the start and goal are in world coordinates, usually meters in the `map` frame.

Example:

- start pose might be `(x=1.2, y=0.8)`
- goal pose might be `(x=4.7, y=2.1)`

But A* does not search nicely in floating-point meters. It searches on grid cells.

Example:

- `(1.2, 0.8)` might become cell `(12, 8)`
- `(4.7, 2.1)` might become cell `(47, 21)`

So you need:

- `worldToGrid(...)`
  Converts meters to cells before planning.

- `gridToWorldPose(...)`
  Converts cells back to meters after planning.

This is one of the most important parts of the planner.

## What A* Should Return

Inside the planner, A* should basically find:

- a list of cells from start to goal

Example:

- `(12, 8)`
- `(13, 8)`
- `(14, 9)`
- `(15, 10)`

Then you convert that list into a ROS path:

- each cell becomes a `PoseStamped`
- all poses go into `nav_msgs::msg::Path`

So the internal result is:

- a list of grid cells

And the ROS-facing result is:

- a `nav_msgs::msg::Path`

## Recommended Coding Order

Follow these steps in order. Do not jump straight into the full A* loop first.

- [ ] **Step 1: Fix the planner class shape**
  In `a_star.hpp` and `a_star.cpp`, make sure `AStarPlanner` is the class being implemented consistently.

  You want these planner entry points:
  - `setMap(...)`
    Return: `void`
  - `hasMap() const`
    Return: `bool`
  - `plan(start, goal)`
    Return: `nav_msgs::msg::Path`

  Why this matters:
  - this gives the action server a clean interface
  - it keeps planner code separate from ROS action code

- [ ] **Step 2: Add a grid coordinate type**
  Create a small type like:
  - `Coordinate { int x; int y; }`

  What it represents:
  - one map cell, not meters

  Why this matters:
  - A* should work on cells, not on `PoseStamped`

- [ ] **Step 3: Add map validation**
  Write `isMapValid() const`.

  Return:
  - `bool`

  This function should check:
  - width > 0
  - height > 0
  - resolution > 0
  - `map_.data.size() == width * height`

  Why this matters:
  - if the map is malformed, nothing else should run

- [ ] **Step 4: Add basic cell helpers**
  Write:
  - `isInBounds(const Coordinate & cell) const`
    Return: `bool`
  - `toIndex(const Coordinate & cell) const`
    Return: `size_t`

  What they do:
  - `isInBounds(...)` checks whether a cell is inside the map
  - `toIndex(...)` converts `(x, y)` into the flat array index for `map_.data`

  Why this matters:
  - occupancy grids store data in a 1D array, so you need a safe way to access cells

- [ ] **Step 5: Add occupancy checking**
  Write `isCellTraversable(const Coordinate & cell) const`.

  Return:
  - `bool`

  Rules for your first version:
  - `map.data >= 50` means blocked
  - `map.data == -1` means blocked
  - anything else is free

  Why this matters:
  - A* needs to know which cells it is allowed to step into

- [ ] **Step 6: Add world-to-grid conversion**
  Write `worldToGrid(double wx, double wy, Coordinate & cell) const`.

  Return:
  - `bool`

  Output parameter:
  - `Coordinate & cell`
    This is where the function writes the converted grid cell if conversion succeeds.

  This function should:
  - read map origin from `map_.info.origin.position`
  - read resolution from `map_.info.resolution`
  - compute the integer cell that contains `(wx, wy)`
  - return `false` if the point falls outside the map

  What it means:
  - this turns the ROS goal into something A* can search on

- [ ] **Step 7: Add grid-to-world conversion**
  Write `gridToWorldPose(const Coordinate & cell, const std_msgs::msg::Header & header) const`.

  Return:
  - `geometry_msgs::msg::PoseStamped`

  This function should:
  - convert a grid cell back into a real-world position in meters
  - place the pose at the center of the cell
  - copy the provided header into the returned pose

  Why this matters:
  - A* finds cells, but your controller and ROS tools want a `Path`

- [ ] **Step 8: Add the heuristic**
  Write `heuristic(const Coordinate & a, const Coordinate & b) const`.

  Return:
  - `double`

  Use Euclidean distance for v1.

  What it means:
  - this is the "guess" A* uses for how far a cell is from the goal

  Why this matters:
  - without it, you do Dijkstra-like exploration instead of A*

- [ ] **Step 9: Add neighbor generation**
  Write `getNeighbors(const Coordinate & cell) const`.

  Return:
  - `std::vector<Coordinate>`

  Use 8-connected motion:
  - left
  - right
  - up
  - down
  - 4 diagonals

  Why this matters:
  - this defines how the robot is allowed to move across the grid

- [ ] **Step 10: Add diagonal movement rules**
  Decide how diagonals are allowed.

  Recommended:
  - do not allow corner-cutting
  - if moving diagonally, both touching side cells should also be free

  Why this matters:
  - otherwise the planner can "slip" through obstacle corners unrealistically

- [ ] **Step 11: Add path reconstruction**
  Write `reconstructPath(...) const`.

  Return:
  - `std::vector<Coordinate>`

  What it does:
  - after A* reaches the goal, you walk backward through parent pointers
  - then reverse that list so it goes start -> goal

  Why this matters:
  - A* does not directly output the whole path unless you reconstruct it

- [ ] **Step 12: Add path message building**
  Write `buildPathMessage(...) const`.

  Return:
  - `nav_msgs::msg::Path`

  This function should:
  - create `nav_msgs::msg::Path`
  - set the header
  - convert each cell to a `PoseStamped`
  - push all poses into `path.poses`

  Why this matters:
  - this is the final translation from planner output to ROS output

- [ ] **Step 13: Build the beginning of `plan(...)`**
  In `plan(...)`, first do the easy checks:
  - confirm a map exists
  - confirm the map is valid
  - choose the output frame from goal header, then start header, then `"map"`

  Why this matters:
  - it keeps failure handling clean before the search starts

- [ ] **Step 14: Convert start and goal into cells**
  In `plan(...)`:
  - convert `start.pose.position.x/y` into `start_cell`
  - convert `goal.pose.position.x/y` into `goal_cell`

  Return an empty path if conversion fails.

  Why this matters:
  - if conversion fails, the goal is outside the known map

- [ ] **Step 15: Reject impossible requests early**
  Still in `plan(...)`, return an empty path if:
  - start is blocked
  - goal is blocked
  - start is outside the map
  - goal is outside the map

  Why this matters:
  - no reason to run A* on invalid input

- [ ] **Step 16: Add A* search data structures**
  Add the containers A* needs:
  - open set priority queue
  - best known `g` score
  - parent map
  - closed/visited tracking

  What they mean:
  - open set = cells you still might explore
  - `g` score = real cost from start to this cell
  - parent map = how you got to this cell
  - closed set = cells already fully processed

- [ ] **Step 17: Initialize the search**
  Put the start cell into the open set with:
  - `g = 0`
  - `f = heuristic(start, goal)`

  Why this matters:
  - this is how the search begins

- [ ] **Step 18: Implement the main A* loop**
  The loop should:
  - pop the best cell from the open set
  - stop if it is the goal
  - get its neighbors
  - skip neighbors that are invalid or blocked
  - compute tentative cost
  - update parent and score if this route is better

  This is the core of the planner.

- [ ] **Step 19: Handle the no-path case**
  If the open set becomes empty before reaching the goal:
  - return an empty path

  What it means:
  - the planner searched everything reachable and found no route

- [ ] **Step 20: Handle the success case**
  If the goal is reached:
  - reconstruct the cell path
  - convert it into a ROS path
  - return it

  This is the successful planner output.

- [ ] **Step 21: Keep pose orientation simple**
  For your first version:
  - intermediate poses can use identity orientation
  - final pose can copy the goal orientation

  Why this matters:
  - you are not doing heading-aware A* yet
  - pure pursuit can follow the path without requiring full `(x, y, theta)` search

- [ ] **Step 22: Keep the action server thin**
  `a_star_action_server.cpp` should only:
  - receive the goal
  - verify a map exists
  - call `planner_.setMap(current_map_)`
  - call `planner_.plan(goal->start, goal->goal)`
  - fill the result

  Do not put A* logic in the action server.

- [ ] **Step 23: Improve feedback**
  Update action feedback so it reflects real planner stages:
  - `"Planning request received"`
  - `"Converting world coordinates to grid"`
  - `"Running A* search"`
  - `"Path found"`
  - `"No path found"`

  Why this matters:
  - it makes debugging much easier while you build

- [ ] **Step 24: Test coordinate conversion before full A***
  Before trusting the full planner, manually test:
  - known world point -> cell
  - same cell -> world point
  - outside-map point -> failure

  Why this matters:
  - if conversion is wrong, A* will look wrong even if the search is correct

- [ ] **Step 25: Test A* on a simple case**
  Use a small easy scenario and verify:
  - free start and free goal produce a path
  - blocked goal fails
  - unknown goal fails
  - diagonal path works

  Why this matters:
  - this isolates search correctness from bigger ROS issues

- [ ] **Step 26: Test the full ROS action flow**
  Verify:
  - action fails if no map has been received
  - action succeeds when a map exists
  - returned path frame is correct
  - `success` matches whether a path was found

## What You Are Building In One Sentence

You are building a planner that takes a start pose and goal pose in meters, converts them into map cells, searches for a collision-free route across the occupancy grid using A*, and returns that route as a ROS `nav_msgs/Path`.

## Defaults Already Chosen

- [x] Use the action `start` pose, not `/estimated_pose`, for v1
- [x] Keep ROS inputs and outputs in meters
- [x] Convert to cells inside the planner
- [x] Use 8-connected A*
- [x] Treat unknown cells as blocked
- [x] Keep heading out of the search state for now
- [x] Preserve goal orientation only in the returned path
