# Paesano V2

## Hardware
- Add one tilt servo, a PCA9685 PWM board, and a camera mount.
- Connect the PCA9685 over I2C through a separate ESP32 or Nano. Keep servo control separate from the motor-control Pi Pico.
- Use the robot's existing 7.4 V (2S) LiPo as the power source. Feed a separate buck regulator from the battery and connect its output to the PCA9685 servo-power input (`V+`). Set the buck output to the servo's rated voltage and size it for the servo's peak or stall current; a full 2S pack reaches about 8.4 V.
- Supply PCA9685 logic power (`VCC`) at the I2C controller's logic voltage (3.3 V for the Pi or ESP32; check level shifting if using a 5 V Nano). Connect the battery, buck, controller, and PCA9685 grounds. Do not power the servo from the Pi's 5 V pin or connect the raw LiPo to `VCC`.

## Software

### Camera geometry
- During mapping, normally point the RGB-D camera downward. Filter out floor points and project nearby obstacle points into the existing occupancy or local obstacle map. Do not keep a second full-size occupancy grid for camera data.
- Use the camera's current pose in TF when projecting depth points into the map.

### Semantic scans
- Trigger a settled image scan after a configurable amount of translation or rotation, with an option to scan when entering a new open space.
- Send the image to Gemini for object descriptions and reported room-type confidence. Treat those scores as model reports, not calibrated probabilities.
- Store each scan as a sparse observation: map position and orientation, timestamp, pose uncertainty when available, object evidence, room-type scores, and a reference to its image if retained. Keep observations separate so later scans can correct earlier guesses.
- Avoid counting repeated views from nearly the same pose as independent evidence.

### Sparse semantic graph and heatmap
- Graph is not a graph, we will be doing nodes, and when quiery kitchen or living room then just do a spread gaussian and sum and take arg max.
- also travel cost
- Make each semantic scan an observation node. Connect nearby nodes when a traversable path through known free space links them. Walls block connections; doorway connections are weaker so evidence does not freely spread between rooms.
- Combine evidence across connected nodes to form provisional room sections and labels while mapping. Revisit node positions after SLAM corrections so the graph remains aligned with the occupancy map.
- Store the observation nodes, edges, and section labels. The heatmap is a visualization rendered from this sparse graph over the existing map, not another persistent N x N grid.
- To query an arbitrary map point, first check that it lies in known free space. Find nearby observation nodes by traversable-path distance through the existing occupancy map, then combine their section evidence with distance and connection weights. Return the strongest section and its score, or `unknown` when evidence is absent or ambiguous. A graph has no literal inside boundary by itself.
- Once structural room boundaries and stable region IDs exist, associate graph sections with those regions. Use the region boundary for an exact inside/outside test and keep the graph as the semantic evidence behind each room label.

semantic_node = {
    "x": 4.2,
    "y": 2.8,
    "scores": {
        "kitchen": 0.85,
        "bedroom": 0.05,
        "living_room": 0.10
    }
}

do slightly before so in free space
make the map relatively coarse
