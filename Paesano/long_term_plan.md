# Paesano Long-Term Plan

## Vision

Evolve Paesano from a metric navigation robot into an embodied intelligent agent that can:

- Understand rooms, hallways, doorways, objects, and named places.
- Navigate using semantic instructions such as `go to my room`.
- Observe and remember events over time.
- Maintain a stable identity, values, relationships, drives, and affective state.
- Reflect on experience and select high-level intentions using a local LLM.
- Express a consistent personality through speech and behavior.
- Remain safe and operational when the LLM is unavailable or produces an invalid action.

The internal system may be named **Consciousness**, but technically it is an embodied cognitive architecture. It must not claim biological or philosophical consciousness.

The semantic-mapping work in [`plan.md`](plan.md) is the prerequisite for this project. The intelligence layer should reason about semantic concepts and validated robot state, not raw motor commands or ungrounded text.

---

## Core Design Principle

The LLM never controls the motors directly.

```text
Sensors and robot state
        ↓
Perception and semantic mapping
        ↓
Paesano Mind
        ↓
Structured intention
        ↓
Policy and safety validation
        ↓
Deterministic ROS 2 orchestrator
        ↓
Planner, controller, and hardware
```

Localization, collision stopping, path planning, motor control, emergency stop, and command validation remain deterministic. The mind layer can propose actions, but the orchestrator decides whether they are valid and safe to execute.

---

## Cognitive Cycle

The entire intelligence layer revolves around one readable executive loop:

```text
Observe
  ↓
Update working memory
  ↓
Select what deserves attention
  ↓
Retrieve relevant memories
  ↓
Appraise the situation and update affect
  ↓
Use rules and/or an LLM to propose an intention
  ↓
Validate the intention against schemas, policies, and safety state
  ↓
Send the accepted intention to the ROS 2 orchestrator
  ↓
Record the resulting experience
```

The loop should be event-driven. It should not continuously invoke an LLM when nothing meaningful has changed.

---

## Proposed Repository Structure

```text
paesano_mind/
├── config/
│   ├── identity.yaml
│   ├── values.yaml
│   ├── drives.yaml
│   └── policies.yaml
│
├── consciousness/
│   ├── cognitive_cycle.py
│   ├── workspace.py
│   ├── attention.py
│   ├── inner_monologue.py
│   └── experience_stream.py
│
├── memory/
│   ├── schemas.py
│   ├── working_memory.py
│   ├── episodic_memory.py
│   ├── semantic_memory.py
│   ├── autobiographical_memory.py
│   ├── retrieval.py
│   └── consolidation.py
│
├── affect/
│   ├── state.py
│   ├── drives.py
│   └── appraisal.py
│
├── identity/
│   ├── self_model.py
│   ├── relationships.py
│   └── values.py
│
├── cognition/
│   ├── prompt_builder.py
│   ├── deliberation.py
│   ├── reflection.py
│   ├── intention.py
│   └── tool_schema.py
│
├── embodiment/
│   ├── perception_interface.py
│   ├── intention_gateway.py
│   └── speech_interface.py
│
├── llm/
│   ├── client.py
│   ├── model_config.yaml
│   └── structured_output.py
│
├── simulated_world/
├── storage/
│   └── mind.db
└── tests/
```

Do not create every abstraction immediately. Add a module only when a working data flow requires it.

---

## Configuration

YAML stores relatively stable configuration, not the robot's entire memory.

Example:

```yaml
identity:
  name: Paesano
  description: Indoor mobile robot

values:
  human_safety: 1.0
  honesty: 0.9
  obedience: 0.8
  curiosity: 0.4

personality:
  verbosity: 0.3
  humor: 0.5
  initiative: 0.6

policies:
  never_publish_cmd_vel_directly: true
  never_bypass_collision_stop: true
  ask_before_entering_private_room: true
```

Values and policies are loaded as constraints. An LLM cannot rewrite them during normal operation.

---

## Memory Architecture

Use SQLite for persistent memory. YAML is not appropriate for a growing, queried, concurrent experience store.

### Working Memory

- Current task and goal.
- Current semantic location.
- Nearby people and objects.
- Orchestrator and navigation state.
- Recent observations and dialogue.
- Small bounded capacity with expiration.

### Episodic Memory

Append-only records of events:

```text
id
timestamp
event_type
source
robot_pose
semantic_location
entities
content
confidence
importance
result
```

Examples include receiving a command, entering a room, encountering an obstacle, completing a task, meeting a person, or failing to plan.

### Semantic Memory

Persistent facts extracted from observations and episodes:

```text
subject
relation
object
confidence
source_episode_ids
created_at
updated_at
```

Example: `Joseph → prefers_destination → desk`.

Facts retain provenance and confidence. An LLM-generated claim does not silently become ground truth.

### Autobiographical Memory

Selected summaries of meaningful episodes, linked back to their original records. Summaries improve retrieval but never replace raw history.

### Consolidation

Periodic background processing should:

- Deduplicate similar events.
- Group events into episodes.
- Create candidate semantic facts.
- Generate summaries.
- Decay or archive low-value working memories.
- Preserve high-importance experiences.
- Require confirmation for uncertain personal facts.

---

## Attention and Global Workspace

Every observation receives a salience score based on:

- Safety urgency.
- Relevance to the current task.
- Novelty.
- Emotional appraisal.
- User involvement.
- Memory importance.

Only the highest-priority information enters the active workspace and LLM context. Safety events always outrank dialogue, curiosity, and personality behavior.

---

## Affect, Drives, and Mood

Affect is represented as bounded numeric state:

```yaml
curiosity: 0.35
confidence: 0.72
social_engagement: 0.60
frustration: 0.08
urgency: 0.10
```

Deterministic appraisal rules update these values:

- Repeated planning failure increases frustration.
- Successful task completion increases confidence.
- An unfamiliar semantic region increases curiosity.
- A low battery increases urgency.
- A person requesting interaction increases social engagement.

Affect influences attention, language, and optional behavior. It never overrides safety policies.

---

## Identity and Relationships

The self-model tracks stable facts about Paesano:

- Physical capabilities and limitations.
- Current software capabilities.
- Known sensors and actuators.
- Current location and task.
- Past validated accomplishments.
- Explicit uncertainty about unavailable information.

Relationship records may store names, interaction history, preferences, trust, and boundaries. Personal information requires provenance, confidence, correction, and deletion support.

---

## Intention Interface

The LLM returns schema-constrained intentions rather than free-form robot commands.

Example:

```json
{
  "tool": "navigate_to",
  "arguments": {
    "place": "my_room"
  },
  "reason": "Joseph requested it",
  "confidence": 0.94
}
```

Initial allowlisted tools:

```text
navigate_to(place)
wait(duration)
stop_navigation()
find_object(object_type)
describe_surroundings()
remember(content)
recall(query)
ask_user(question)
speak(text)
```

The intention gateway validates:

- Tool name is allowlisted.
- Arguments satisfy a strict schema.
- Named places and objects exist.
- Robot state permits the action.
- Safety policies allow the action.
- Required confirmation has been received.
- Rate and resource limits are satisfied.

The mind process has no direct `/cmd_vel` publisher and no shell-execution tool.

---

## ROS 2 Interfaces

Likely perception inputs:

```text
/estimated_pose
/semantic_map
/semantic_location
/detected_objects
/orchestrator/state
/dynamic_obstacle_blocked
/battery_state
/speech/transcript
```

Likely outputs:

```text
/navigation/named_goal
/mind/state
/mind/intention
/mind/attention
/speech/say
```

The orchestrator remains the sole owner of navigation lifecycle decisions.

---

## Local LLM Runtime

Start with a small quantized instruction model through `llama.cpp` or Ollama.

Requirements:

- Structured JSON output.
- Short bounded context assembled by the mind layer.
- Event-driven inference.
- Request timeout and cancellation.
- Deterministic fallback when inference fails.
- Measured CPU, RAM, temperature, latency, and navigation-loop impact.

If local inference degrades localization, LiDAR processing, control timing, or thermal stability, move inference to a separate laptop or mini-PC. All safety-critical behavior remains on the robot.

---

## Simulated World

Create a deterministic harness that feeds synthetic observations and robot state into the mind without physical motion.

Test scenarios:

- A temporary obstacle blocks a path.
- Replanning fails repeatedly.
- A requested place does not exist.
- The LLM emits malformed JSON.
- The LLM requests a forbidden tool.
- A memory contradicts a newer correction.
- A person gives an ambiguous command.
- Inference times out during navigation.
- Battery becomes critically low during a task.

The simulation validates intentions and state transitions without allowing motor commands.

---

## Development Roadmap

### Stage 1 — Semantic Embodiment

- Complete geometric room and hallway segmentation.
- Detect and localize doorways.
- Build the semantic topological graph.
- Add room and hallway visual classification.
- Add object detection and RGB-D localization.
- Implement named-place navigation.

### Stage 2 — Experience Stream

- Define observation and event schemas.
- Subscribe to semantic and navigation state.
- Record an append-only event stream in SQLite.
- Build event inspection and replay tooling.

### Stage 3 — Memory

- Implement bounded working memory.
- Add episodic queries by time, place, entity, and event type.
- Add semantic facts with provenance and confidence.
- Add retrieval tests.

### Stage 4 — Deterministic Executive

- Implement the cognitive cycle without an LLM.
- Add attention scoring.
- Add rule-based appraisal and affect.
- Implement the intention gateway and allowlisted tools.
- Exercise the system in the simulated world.

### Stage 5 — LLM Deliberation

- Integrate a local model runtime.
- Build bounded prompt assembly from workspace and retrieved memories.
- Require schema-constrained outputs.
- Add timeout, malformed-output, and hallucinated-tool handling.
- Compare LLM decisions against deterministic baselines.

### Stage 6 — Identity and Social Memory

- Add the self-model, values, and personality configuration.
- Add relationship records and user corrections.
- Add autobiographical summaries with source links.
- Add privacy and deletion mechanisms.

### Stage 7 — Natural Interaction

- Add speech recognition and text-to-speech.
- Add named-place voice commands.
- Add status explanations and clarification questions.
- Add controlled inner monologue and reflection.

### Stage 8 — Consolidation and Imagination

- Consolidate episodes into long-term memories.
- Add offline reflection over completed tasks.
- Add a bounded simulated-world planner for proposed intentions.
- Never treat imagined events as real memories.

---

## Evaluation

### Semantic Perception

- Room/hallway classification F1.
- Doorway precision and recall.
- Doorway map-position error.
- Topological graph node and edge accuracy.
- Inference latency and resource use.

### Memory

- Retrieval precision and recall on scripted queries.
- Fact provenance coverage.
- Contradiction and correction handling.
- Consolidation compression ratio.
- False-memory rate.

### Cognition

- Task-completion rate.
- Invalid-tool proposal rate.
- Safety-gate rejection rate.
- Clarification rate on ambiguous commands.
- Decision latency.
- Recovery from model timeout or failure.

### Systems Safety

- Navigation-loop frequency during inference.
- CPU, RAM, and temperature under load.
- Emergency-stop latency.
- Behavior with the mind process terminated.
- Zero direct motor commands from the mind layer.

All resume claims must be tied to a documented measurement method and completed validation.

---

## Non-Goals

- Claiming biological consciousness or sentience.
- Allowing an LLM to control wheel velocities.
- Treating generated summaries as unquestionable facts.
- Giving the model unrestricted shell or filesystem access.
- Building every cognitive module before a minimal end-to-end loop works.
- Sacrificing navigation reliability for conversational behavior.

---

## Long-Term Demonstration

A complete demonstration should show:

1. Paesano recognizes that it is in a hallway.
2. It detects and maps the doorway to a known room.
3. A user says, `Go to my room and look for my backpack.`
4. The mind resolves the named location and proposes a structured intention.
5. The orchestrator validates and executes navigation.
6. Paesano waits for temporary obstacles and replans around persistent ones.
7. It identifies and localizes the requested object.
8. It reports the result through speech.
9. It records the task as an episode.
10. Later, it can accurately recall what happened, where, and with what confidence.

This demonstrates perception, semantic mapping, localization, planning, memory, local-LLM inference, safety architecture, and measurable embodied autonomy as one coherent system.

---

## Future Resume Positioning

Only after implementation and validation, the project may support a claim such as:

> Designed an embodied cognitive architecture integrating semantic mapping, persistent episodic and semantic memory, affective state, and schema-constrained local-LLM planning with a deterministic ROS 2 safety layer.

Replace general language with verified model, latency, accuracy, navigation, and memory metrics when available.
