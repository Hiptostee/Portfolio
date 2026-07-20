# Minimal Alive Paesano

## Goal

By Christmas break 2026, turn Paesano into a minimal embodied companion that can explore,
see, understand named places, talk, remember grounded experiences, and safely act on simple
requests. It should feel continuous across conversations and restarts rather than resetting
into a blank robot each time.

## Christmas 2026 Definition of Done

Paesano can:

- [ ] Autonomously explore an unknown indoor environment and save its map.
- [ ] Divide the map into stable rooms, hallways, and connected regions.
- [ ] Automatically classify common room types and remember each classification with its evidence and confidence.
- [ ] Let a user name or correct places without erasing the original classification history.
- [ ] Know and report its current named place and inferred room type.
- [ ] Detect and map a small set of useful objects with an RGB-D camera.
- [ ] Hear speech and answer through a speaker.
- [ ] Remember conversations, places visited, objects seen, and task results across restarts.
- [ ] Answer questions about its grounded memories with time, place, and uncertainty.
- [ ] Accept simple requests such as `go to my room` and `where did you see my backpack?`.
- [ ] Route every physical action through the deterministic orchestrator and safety checks.
- [ ] Complete a repeatable end-to-end hardware demonstration.

## Near-Term Wednesday Milestones

This dated schedule is the active priority order as of Saturday, July 18, 2026. Keep each milestone
to its minimum working system and move unfinished hardening into `AUDITS.md` rather than expanding
the feature midweek.

### Wednesday, July 22 — Autonomous Mapping

- [ ] Implement frontier detection, clustering, safe-goal selection, scoring, and spatial failed-goal filtering.
- [ ] Generate standoff poses by moving away from nearby unknown cells, then require the candidate
  cell to be traversable in the latest geometry-compatible `/map_inflated`.
- [ ] Report recoverable `STUCK` when frontiers exist but all bounded approach candidates are blocked
  or blacklisted; reserve `COMPLETE` for repeated updates with no retained frontier clusters.
- [ ] Build and launch the mapping pose, A*, LQR, orchestrator, and explorer component flow.
- [ ] Explore an unknown simulated map without manual driving.
- [ ] Avoid immediately retrying a frontier after A* rejects its goal.
- [ ] Stop when no meaningful reachable frontiers remain and save the resulting map.
- [ ] Complete at least one supervised physical-robot exploration run if simulation is repeatable.

Definition of done: Paesano repeatedly explores and saves a usable simulated map on its own. A
supervised hardware run is the target, but reliability hardening remains tracked in `AUDITS.md`.

### Immediately After Autonomous Mapping — Adaptive Planning Map

- [ ] Track time-gated occupied and clear evidence for globally aligned cells observed in `/local_map`.
- [ ] Preserve immediate local-obstacle stopping without waiting for persistent confirmation.
- [ ] Add a validated clear mask that can override stale occupied furniture in `/planning_map`
  without changing the saved localization `/map`.
- [ ] Keep newly observed persistent obstacles occupied across local scans.
- [ ] Publish planning-map changes only when cell state changes or at a bounded rate.
- [ ] Trigger or retry planning when a relevant map cell changes traversability.
- [ ] Demonstrate that removing an object captured in the saved map opens the route, while placing a
  new object still stops the robot and produces a safe replan.

Definition of done: Paesano adapts navigation to a moved dorm-room obstacle without rewriting or
destabilizing its localization map. Database persistence and versioned map consolidation are not
required for this first implementation.

### Wednesday, July 29 — Room Segmentation

- [ ] Create `paesano_semantic_mapping`.
- [ ] Segment occupancy-grid free space into room, hallway, doorway, and unknown regions.
- [ ] Assign stable region IDs and safe representative poses.
- [ ] Publish region boundaries and labels for RViz inspection.
- [ ] Save and reload the region representation with the map.

Definition of done: a saved indoor map is divided into recognizable rooms and hallways with stable
IDs that survive reload. Learned room-purpose classification is not required yet.

### Wednesday, August 5 — RGB-D and YOLO

- [ ] Bench-test the D435 RGB, infrared, depth, and aligned-depth streams before the final mount exists.
- [ ] Run initial YOLO and RGB-D projection experiments with the camera secured on a temporary fixed support.
- [ ] Launch and calibrate the selected RGB-D camera relative to `base_link`.
- [ ] Run a pretrained YOLO model on a small room-evidence vocabulary.
- [ ] Reject low-confidence detections and invalid depth.
- [ ] Project detections into the `map` frame.
- [ ] Associate detections with the correct segmented region.
- [ ] Demonstrate at least one object observation contributing evidence toward a room type.

Definition of done: Paesano does more than draw bounding boxes; it places a detected object in the
map and associates it with a stable room region.

### Thursday, August 6 onward — Memory SDK

Begin the separate local-first memory library: SQLite migrations, append-only episodes, semantic
facts, provenance, confidence, correction history, structured retrieval, and bounded LLM context.
Integrate it with Paesano only after the core API works independently.

## Detailed Build Scope

The sections below retain the broader subsystem details. The dated milestone order above controls
the immediate implementation sequence.

### Autonomous Mapping — Detailed Scope

- [ ] Run A*, trajectory following, and the orchestrator while `slam_toolbox` is mapping.
- [ ] Provide the robot's current `map`-frame pose during mapping.
- [ ] Create a `paesano_explorer` ROS 2 package.
- [ ] Detect, cluster, filter, and score frontier regions.
- [ ] Estimate the inward direction from nearby unknown cells and generate a bounded set of
  spatially separated standoff candidates for each retained cluster.
- [ ] Validate candidate geometry and traversability against `/map_inflated`, then publish the best
  safe frontier through `/navigation/goal`.
- [ ] Report explicit navigation success, failure, and cancellation results.
- [ ] Temporarily blacklist unreachable frontiers and select another target.
- [ ] Distinguish recoverable `STUCK` from confirmed `COMPLETE`; save the map automatically only
  after repeated updates contain no retained frontier clusters.
- [ ] Visualize frontiers and the selected goal in RViz.
- [ ] Test with synthetic grids, simulation, and the physical robot.

Acceptance test: starting from an unknown map, Paesano explores without manual driving,
recovers from an unreachable frontier, declares completion, and saves a usable map.

### Adaptive Planning Map — Detailed Scope

- [ ] Maintain a grid aligned to `/map` with occupied score, clear score, state, and observation time.
- [ ] Accept at most one evidence vote per cell per scan and gate votes by elapsed time or viewpoint.
- [ ] Use free LiDAR rays to clear stale saved-map occupancy only after consistent evidence.
- [ ] Let any current local obstacle override clear or free state immediately.
- [ ] Fuse the saved map, persistent-change state, clear mask, and current local obstacles into
  `/planning_map`.
- [ ] Keep the particle filter subscribed to the unchanged saved `/map`.
- [ ] Apply hysteresis so cells do not alternate rapidly between occupied and free.
- [ ] Bound planning-map publication and A* inflation work.
- [ ] Preserve an active goal or safely retry it when changed traversability makes a route possible.
- [ ] Visualize changed-occupied and validated-clear cells separately in RViz.

Acceptance test: a stale mapped obstacle can become traversable after repeated valid clear rays;
a new or returning obstacle blocks immediately; localization remains stable throughout.

### Places and Spatial Understanding — Detailed Scope

- [ ] Create a small `paesano_semantic_mapping` package.
- [ ] Segment free space into stable regions using occupancy-grid geometry.
- [ ] Detect room, hallway, and doorway structure from geometry.
- [ ] Assign stable region IDs and safe navigation poses.
- [ ] Create an initial geometric classification for each region: room, hallway, doorway, or unknown.
- [ ] Let the user name or correct regions while retaining confidence and correction history.
- [ ] Persist region IDs, boundaries, names, classifications, confidence, evidence, and safe poses.
- [ ] Publish `/semantic_location` from the robot's current pose.
- [ ] Navigate to a region ID or user-confirmed place name.

Acceptance test: Paesano distinguishes rooms from hallways, remembers those classifications
after restart, and can report and navigate to a user-named place such as `my room`.

### Minimal Persistent Memory and Text Conversation — Starts August 6, 2026

- [ ] Create a minimal `paesano_mind` package and SQLite database with migrations.
- [ ] Implement bounded working memory for the current conversation and task.
- [ ] Store append-only episodes for speech, place transitions, navigation results, and observations.
- [ ] Store semantic facts separately with confidence, provenance, and correction support.
- [ ] Persist user-confirmed facts such as `my room refers to Region 4`.
- [ ] Persist inferred facts such as `Region 4 is probably a bedroom` with supporting observations.
- [ ] Retrieve both the current room classification and its previous corrections after restart.
- [ ] Retrieve memories by time, place, entity, and event type.
- [ ] Build a text-only conversation loop before adding microphones and speakers.
- [ ] Give the language model only a bounded set of retrieved memories and verified robot state.
- [ ] Clearly distinguish observed facts, user statements, model inferences, and uncertainty.
- [ ] Add a memory inspector so stored records can be reviewed, corrected, or deleted.

Acceptance test: after a restart, Paesano can accurately answer where it went, whether a
task succeeded, and what Joseph previously told it, while citing the relevant time or place.

### Eyes and Object Memories — YOLO Milestone August 5, Continued After

- [ ] Use one D435 as Paesano's RGB-D observation camera rather than maintaining two camera pipelines.
- [ ] Verify the complete camera and YOLO pipeline on a bench before waiting for the final CAD mount.
- [ ] Start with a secured adjustable fixed mount so active-vision mechanics do not block RGB-D localization.
- [ ] After measuring the physical camera, CAD a rigid elevated holder with a single tilt axis; the robot provides horizontal rotation.
- [ ] Add repeatable down, forward, and up observation poses for the floor ahead, normal viewing, and higher objects.
- [ ] Publish the tilt joint state and model `base_link -> camera_tilt_link -> camera_link` in TF.
- [ ] Stop the robot, command a view, wait for the camera and servo to settle, and only then create mapped observations.
- [ ] Mount and calibrate the RGB-D camera to `base_link` through its current tilt transform.
- [ ] Detect a focused room-evidence vocabulary: person, backpack, chair, bed, couch, TV,
  refrigerator, microwave, sink, toilet, desk, and door.
- [ ] Convert image detections and depth into `map`-frame positions.
- [ ] Reject invalid depth and low-confidence detections.
- [ ] Track repeated observations instead of creating duplicate objects.
- [ ] Associate each observation with a semantic region and timestamp.
- [ ] Store object sightings as episodes; store `last seen` as a derived fact with provenance.
- [ ] Fuse observed objects with geometry to classify bedroom, kitchen, living room, hallway, and unknown.
- [ ] Recompute classification confidence as new evidence arrives without overwriting confirmed labels.
- [ ] Persist the evidence behind each classification, such as `bed + geometry -> bedroom`.
- [ ] Add `describe_surroundings` and `recall_last_seen(object_type)` capabilities.
- [ ] Measure detection precision, map-position error, and room-classification accuracy on a small labeled test set.

Acceptance test: Paesano sees a bed and other evidence, classifies the containing region as a
bedroom, remembers that classification after restart, and answers where it last saw a backpack.

### Voice and Personality — October 2026

- [ ] Add push-to-talk or wake-word-controlled speech recognition.
- [ ] Add text-to-speech through the robot's speaker.
- [ ] Feed transcripts into the already-tested text conversation loop.
- [ ] Keep a consistent name, concise speaking style, and a few bounded personality traits.
- [ ] Ask clarification questions for ambiguous places, objects, or commands.
- [ ] Let emergency stop and navigation status interrupt speech and cognition.
- [ ] Prevent generated speech from being stored automatically as a real observation or user fact.

Acceptance test: Joseph can hold a short spoken conversation, teach Paesano a place name,
restart it, and later ask Paesano to recall that information aloud.

### Safe Language-Model Actions — November 2026

- [ ] Define schema-constrained tools: `navigate_to`, `stop_navigation`, `describe_surroundings`,
  `recall`, `propose_memory_update`, `correct_memory`, `ask_user`, `speak`, and `wait`.
- [ ] Validate tool names, arguments, robot state, semantic targets, and safety policy.
- [ ] Let Paesano propose grounded memory updates from conversation and perception.
- [ ] Store the source, timestamp, confidence, and superseded fact for every memory correction.
- [ ] Require Joseph's confirmation before changing important or uncertain personal and spatial facts
  or before entering restricted places.
- [ ] Route accepted navigation intentions through the deterministic orchestrator.
- [ ] Ensure the mind has no `/cmd_vel` publisher and cannot bypass collision stopping.
- [ ] Add inference timeouts, cancellation, malformed-output rejection, and deterministic fallback.
- [ ] Measure inference latency, CPU, RAM, temperature, and navigation-loop impact.
- [ ] Test invented tools, unknown places, contradictory memories, and model shutdown.

Acceptance test: natural-language requests can trigger only allowlisted, validated actions;
Paesano remains safely operable with the language model stopped or producing invalid output.

### Integration and Christmas Demonstration — December 2026

- [ ] Run the complete system repeatedly in simulation before hardware testing.
- [ ] Create rosbag scenarios for memory, perception, navigation, and failure regression tests.
- [ ] Run multiple end-to-end hardware trials and record completion and failure rates.
- [ ] Verify that memories survive restart and remain tied to their original observations.
- [ ] Verify emergency-stop behavior and zero direct motor commands from the mind process.
- [ ] Record CPU, RAM, temperature, inference latency, navigation reliability, and memory accuracy.
- [ ] Document setup, architecture, test methods, limitations, and recovery procedures.
- [ ] Record the final demonstration.

Final demonstration:

1. Paesano autonomously explores and saves an unfamiliar map.
2. Paesano segments rooms and hallways and classifies one room from geometry and observed objects.
3. Joseph confirms or corrects that room and names it `my room`.
4. Paesano detects a backpack there and stores a grounded sighting.
5. After a restart, Joseph asks what kind of room it is, where Paesano went, and where it saw the backpack.
6. Paesano answers aloud with the remembered classification, evidence, place, time context, and uncertainty.
7. Joseph says `go to my room`; the validated intention reaches the deterministic orchestrator.
8. Paesano navigates there, handles a temporary obstacle, reports success, and remembers the task.

## Explicitly After Christmas

- Learned or open-vocabulary room classification beyond the initial geometry-and-object rules.
- Large object vocabularies, custom model training, CLIP, and open-vocabulary vision.
- Autonomous object-search policies beyond reporting the last known location.
- Sophisticated affect, moods, drives, relationship modeling, and inner monologue.
- Autobiographical summarization and large-scale memory consolidation.
- Multi-floor mapping, multiple robots, and unrestricted natural-language task planning.

## Human-Supervised Self-Improvement — After Christmas

Goal: Paesano may diagnose its own failures and prepare tested improvements, but it may never
silently modify or deploy its live safety-critical software.

- [ ] Add developer tools for `create_bug_report`, `collect_diagnostics`, `propose_patch`,
  `run_build`, `run_regression_suite`, `show_diff`, `request_deployment`, and `rollback`.
- [ ] Let Paesano turn navigation, perception, and memory failures into structured bug reports.
- [ ] Attach relevant logs, parameters, maps, memory records, and rosbag time ranges as evidence.
- [ ] Run the coding agent in a separate sandbox, container, and Git branch or worktree.
- [ ] Restrict writable paths and prohibit root access, firmware flashing, and live motor commands.
- [ ] Require the affected packages to build and pass unit, rosbag, and simulation regressions.
- [ ] Produce a human-readable diagnosis, patch diff, risks, test results, and rollback plan.
- [ ] Require Joseph's explicit approval before merge or deployment.
- [ ] Disable motors and preserve a known-good release during deployment and hardware validation.
- [ ] Prevent autonomous edits to emergency-stop, motor-control, safety-policy, and deployment permissions.
- [ ] Compare before-and-after behavior using the same recorded scenario and report whether it improved.
- [ ] Log every proposed, rejected, approved, deployed, and rolled-back change.

Acceptance test: after a reproducible frontier-selection failure, Paesano gathers the relevant
evidence, prepares an isolated patch, passes the regression suite, explains the change, and waits
for approval. Only an approved release is deployed, and the previous release remains recoverable.

## Post-MVP Master Backlog

The original comprehensive checklist remains below as the long-term backlog. It is not the
Christmas MVP schedule; pull items from it only when a phase above needs them.

### 1. Autonomous Mapping and Frontier Exploration

- [ ] Make the navigation stack run while `slam_toolbox` is in mapping mode.
- [ ] Provide the robot's current `map`-frame pose during mapping (from TF or a dedicated pose topic).
- [ ] Create a `paesano_explorer` ROS 2 package.
- [ ] Detect frontier cells: known free cells adjacent to unknown space.
- [ ] Cluster nearby frontier cells into exploration targets.
- [ ] Reject clusters that are too small, obstructed, or outside the traversable map.
- [ ] Score candidates using path distance and expected information gain.
- [ ] Select a safe reachable goal near the best frontier and publish it to `/navigation/goal`.
- [ ] Add explicit navigation results so the explorer can distinguish success, failure, and cancellation.
- [ ] Blacklist unreachable frontiers temporarily and retry them only after the map changes.
- [ ] Add start, pause, resume, cancel, and exploration-complete controls.
- [ ] Prevent exploration goals from overriding manual or emergency commands.
- [ ] Visualize frontier clusters, chosen goals, and blacklisted goals in RViz.
- [ ] Automatically save the completed occupancy map and its metadata.
- [ ] Add the explorer to bringup behind an `exploration_mode` launch argument.
- [ ] Test frontier extraction on synthetic occupancy grids.
- [ ] Demonstrate complete autonomous mapping in simulation, then on the physical robot.

### 2. Geometric Room and Hallway Segmentation

- [ ] Create a `paesano_semantic_mapping` package and define its ROS interfaces.
- [ ] Convert `/map` `OccupancyGrid` messages into an OpenCV representation.
- [ ] Clean map noise and close small wall gaps with morphology.
- [ ] Compute a distance transform and free-space clearance map.
- [ ] Detect room seeds in wide open areas and grow them through free space.
- [ ] Merge duplicate or over-segmented regions.
- [ ] Detect narrow hallways and distinguish them from rooms.
- [ ] Detect doorways and passages between regions.
- [ ] Assign stable region IDs when the occupancy map updates.
- [ ] Publish the segmented semantic map and RViz markers.
- [ ] Save and reload region data alongside the metric map.
- [ ] Measure segmentation and doorway accuracy on labeled test maps.

### 3. Topological Map

- [ ] Define graph nodes for rooms, hallways, and other navigable regions.
- [ ] Define graph edges for doorways and passages.
- [ ] Store each region's boundary, centroid, type, confidence, and safe navigation poses.
- [ ] Build and update the graph automatically from segmentation results.
- [ ] Validate graph connectivity against the occupancy grid.
- [ ] Publish, visualize, save, and reload the graph.
- [ ] Support navigation to a region ID instead of only an `(x, y)` coordinate.

### 4. Camera Perception and Object Mapping

- [ ] Select and mount an RGB-D camera and calibrate it to `base_link`.
- [ ] Launch the camera driver and verify synchronized RGB, depth, and camera info.
- [ ] Add YOLO object detection and publish class, confidence, and bounding boxes.
- [ ] Convert detection pixels and depth into 3D camera-frame positions.
- [ ] Transform object positions into the `map` frame with TF.
- [ ] Filter invalid depth and reject low-confidence observations.
- [ ] Track repeated observations so one object does not become many map entries.
- [ ] Assign each object to its containing semantic region.
- [ ] Persist object class, position, region, confidence, and observation time.
- [ ] Visualize mapped objects in RViz.
- [ ] Measure object detection, localization, and region-assignment accuracy.

### 5. Semantic Region Classification

- [ ] Classify rooms using geometry and observed-object evidence.
- [ ] Add initial rules for kitchen, bedroom, living room, hallway, and unknown.
- [ ] Preserve multiple hypotheses and confidence instead of forcing a label.
- [ ] Allow a user to name, rename, or correct a region.
- [ ] Keep user-confirmed names separate from inferred room types.
- [ ] Persist labels, confidence, evidence, and correction history.
- [ ] Publish the robot's current semantic location.
- [ ] Measure room-classification precision and recall.

### 6. Named-Place and Object-Aware Navigation

- [ ] Accept a region ID or place name as a navigation goal.
- [ ] Resolve names to a safe reachable pose inside the requested region.
- [ ] Plan through the topological graph and use metric A* for each path segment.
- [ ] Handle duplicate, unknown, or ambiguous place names.
- [ ] Add `find_object(object_type)` using the persistent object map.
- [ ] Add active search when an object's location is unknown or stale.
- [ ] Report success, failure, cancellation, and confidence to the requester.
- [ ] Demonstrate commands such as `go to the kitchen` and `find my backpack`.

### 7. Experience Stream and Persistent Memory

- [ ] Create the `paesano_mind` package with only the modules needed for an end-to-end loop.
- [ ] Define schemas for observations, events, entities, facts, and intentions.
- [ ] Create the SQLite database and migrations.
- [ ] Record an append-only stream of semantic, navigation, object, dialogue, and safety events.
- [ ] Build bounded working memory with expiration.
- [ ] Add episodic queries by time, place, entity, and event type.
- [ ] Add semantic facts with provenance, confidence, correction, and deletion support.
- [ ] Add autobiographical summaries linked to their original events.
- [ ] Build event inspection, replay, backup, and recovery tools.
- [ ] Test retrieval quality, contradiction handling, and false-memory prevention.

### 8. Deterministic Cognitive Executive and Safety Gateway

- [ ] Implement the event-driven observe-to-intention cognitive cycle without an LLM first.
- [ ] Add attention and salience scoring with safety events at the highest priority.
- [ ] Add bounded affect and drive state updated by deterministic appraisal rules.
- [ ] Define schema-constrained, allowlisted intentions and tool arguments.
- [ ] Validate robot state, semantic targets, policies, confirmations, and rate limits.
- [ ] Route accepted navigation intentions through the deterministic orchestrator.
- [ ] Ensure the mind cannot publish motor commands or bypass collision stopping.
- [ ] Add deterministic fallback behavior when cognition is unavailable.
- [ ] Build a simulated-world harness for malformed, unsafe, ambiguous, and timeout cases.

### 9. Local LLM Deliberation

- [ ] Benchmark a small quantized instruction model with `llama.cpp` or Ollama.
- [ ] Build a bounded prompt from working memory and retrieved relevant memories.
- [ ] Require strict structured output matching the intention schema.
- [ ] Reject malformed output, invented tools, invalid places, and unsafe requests.
- [ ] Add inference timeout, cancellation, rate limiting, and deterministic fallback.
- [ ] Measure latency, CPU, RAM, temperature, and impact on navigation timing.
- [ ] Move inference off the robot computer if it degrades safety-critical processes.
- [ ] Compare LLM decisions with the deterministic baseline on scripted scenarios.

### 10. Identity, Personality, and Relationships

- [ ] Add immutable safety policies and stable values in YAML configuration.
- [ ] Add a self-model containing verified capabilities, limitations, state, and uncertainty.
- [ ] Add bounded personality parameters that affect speech but never safety behavior.
- [ ] Store relationship history and user preferences with provenance and consent.
- [ ] Support correction and deletion of personal information.
- [ ] Prevent the robot from claiming biological consciousness or unverified experiences.

### 11. Speech and Natural Interaction

- [ ] Add speech-to-text and text-to-speech interfaces.
- [ ] Parse voice commands into the same validated intention schema.
- [ ] Ask clarification questions for ambiguous people, places, objects, or requests.
- [ ] Explain current status, failures, uncertainty, and safety rejections.
- [ ] Add controlled reflection and inner-monologue output that cannot execute actions directly.

### 12. Consolidation and Final Demonstration

- [ ] Consolidate raw events into episodes, summaries, and candidate facts in the background.
- [ ] Deduplicate memories and archive low-value records without losing important provenance.
- [ ] Keep imagined or simulated events clearly separated from real memories.
- [ ] Run failure tests with navigation blocked, the LLM stopped, bad output, and low battery.
- [ ] Verify emergency-stop latency and zero direct motor commands from the mind process.
- [ ] Complete the long-term demonstration defined in `long_term_plan.md`.
- [ ] Document all accuracy, latency, reliability, and resource measurements.
- [ ] Record a repeatable demo and update resume claims using only measured results.

Current capabilities:
- Localization
- SLAM
- Occupancy Grid Mapping
- Path Planning
- Dynamic Obstacle Avoidance
- Autonomous Replanning

New capabilities:
- Automatic room segmentation
- Hallway detection
- Topological map generation
- Object-aware semantic mapping
- Named-place navigation

---

# System Architecture

Occupancy Grid
    │
    ▼
Geometry Processing
    │
    ├── Distance Transform
    ├── Connected Components
    ├── Region Growing
    ├── Room/Hallway Classification
    ▼
Semantic Regions
    │
    ▼
Topological Graph

Camera
    │
    ▼
YOLO Object Detection
    │
    ▼
Object Localization (RGB-D + TF)
    │
    ▼
Assign Objects To Regions
    │
    ▼
Semantic Region Labels

Planner
    │
    ▼
Navigate To:
- Kitchen
- Bedroom
- Living Room
- Hallway

---

# Phase 1 — Geometry

Goal:
Automatically segment rooms and hallways from the occupancy grid.

Tasks

- Convert OccupancyGrid → OpenCV image
- Clean map using morphology
- Compute distance transform
- Detect wide open regions
- Grow regions
- Merge regions
- Detect hallways
- Build adjacency graph
- Visualize regions in RViz

Deliverable

Robot automatically discovers

Room 1
Room 2
Hallway 1

without any machine learning.

---

# Phase 2 — Topological Graph

Create graph

Node
- Room
- Hallway

Edge
- Doorway / Passage

Graph enables

navigate(Room 2)

instead of

navigate(x, y)

---

# Phase 3 — Object Detection

Run YOLO

Detect

- Bed
- Couch
- Chair
- TV
- Refrigerator
- Microwave
- Sink
- Person
- Backpack

Publish detections.

---

# Phase 4 — Object Localization

For every detection

Image Pixel

↓

Depth

↓

Camera Frame

↓

TF

↓

Map Frame

↓

Region

Store

Object
Position
Region

---

# Phase 5 — Semantic Room Classification

Each region stores observed objects.

Example

Region 3

Objects

- Refrigerator
- Microwave
- Sink

↓

Kitchen

Bedroom

Objects

- Bed
- Laptop
- Chair

↓

Bedroom

Living Room

Objects

- Couch
- TV

↓

Living Room

---

# Phase 6 — Named Navigation

Examples

navigate("Kitchen")

Robot

↓

Find kitchen region

↓

Plan through topological graph

↓

Drive to safe point inside room

---

# Stretch Goals

- Persistent object map
- Object tracking
- CLIP-based room classification
- Natural language commands
- Search for objects
- Multi-floor semantic maps
- Semantic exploration
