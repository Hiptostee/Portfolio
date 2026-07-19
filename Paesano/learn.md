# Paesano Semantic Mapping — Focused Learning Roadmap

## Starting Point

Paesano is already a functioning autonomous robotics platform.

You already know enough to work with:

- C++
- Python
- ROS 2 nodes, topics, messages, and launch files
- Occupancy grids
- TF usage at a practical robotics level
- LiDAR mapping and localization
- A* and metric path planning
- State estimation
- Trajectory generation and tracking
- RViz debugging
- Onboard Raspberry Pi deployment

You do **not** need another general ROS course or robotics course before starting this extension.

The missing subjects are:

1. Computational geometry
2. Topological mapping
3. Machine learning fundamentals
4. Object detection (YOLO)
5. RGB-D vision
6. Semantic mapping

---

# Module 1 — Image Processing & Computational Geometry

## Why this comes first

An occupancy grid is just an image.

Before you can segment rooms or hallways, you need to understand how to manipulate binary images and extract geometric information from them.

## Learn

### Binary Image Processing

- Binary images
- Thresholding
- Image masks
- Occupied vs free space

### Morphological Operations

- Erosion
- Dilation
- Opening
- Closing
- Structuring elements

Understand **what these operations do geometrically**, not just the OpenCV functions.

### Connected Components

Learn how to:

- Find disconnected regions
- Label regions
- Compute region area
- Compute bounding boxes

### Distance Transform

Learn:

- What a distance transform is
- Why it represents local clearance
- Why room centers have higher values than hallways
- How doors appear as narrow bottlenecks

### Contours

Learn how to compute:

- Region boundary
- Area
- Perimeter
- Centroid

---

## Practice Project

Load an occupancy-grid image.

Implement:

1. Thresholding
2. Morphology
3. Connected components
4. Distance transform

Visualize every intermediate result.

---

## Success Criteria

You can look at an occupancy grid and explain:

- where rooms are,
- where hallways are,
- where bottlenecks are,

using only geometry.

---

# Module 2 — Computational Geometry

## Goal

Describe regions mathematically.

## Learn

### Region Features

- Area
- Perimeter
- Centroid
- Bounding box
- Oriented bounding box
- Aspect ratio
- Elongation
- Compactness

### Region Growing

Learn how to:

- Flood fill
- Grow regions
- Expand from seeds
- Handle competing labels

### Skeletons

Understand:

- Medial axis
- Skeletonization
- Hallway centerlines
- Branch points

### Region Adjacency

Learn:

- Which regions touch
- Shared boundaries
- Narrow passages
- Doorway detection

---

## Practice Project

Given a floorplan:

- Detect rooms
- Detect hallways
- Compute all geometric features
- Classify regions using simple rules

---

## Success Criteria

Represent a region with:

```text
Area
Centroid
Width
Length
Average clearance
Neighbors
```

and explain why those features distinguish rooms from hallways.

---

# Module 3 — Topological Mapping

## Goal

Convert geometric regions into a graph.

Instead of:

```text
Occupancy Grid
```

build:

```text
Bedroom ---- Hallway ---- Kitchen
                  |
                Office
```

## Learn

### Graph Representation

- Region → Node
- Passage → Edge

### Region Adjacency Graphs

Learn how to:

- Build graphs from segmented maps
- Store neighbors
- Store passage locations
- Store traversal costs

### Hierarchical Planning

Understand the difference between:

Metric planning

```text
(x,y)
```

Topological planning

```text
Bedroom
↓

Hallway

↓

Kitchen
```

---

## Practice Project

Build a graph from a manually segmented floorplan.

Have your code output:

```text
Region 0

↓

Region 2

↓

Region 4
```

---

## Success Criteria

Understand why topological planning is easier for high-level navigation than occupancy-grid planning.

---

# Module 4 — Machine Learning Fundamentals

## Goal

Understand enough ML to confidently use and fine-tune YOLO.

You do **NOT** need a graduate ML course.

## Learn

### Supervised Learning

- Inputs
- Labels
- Predictions
- Loss
- Training
- Validation
- Testing
- Overfitting

### Neural Networks

Conceptually understand:

- Layers
- Weights
- Activations
- Forward pass
- Backpropagation
- Gradient descent

### CNNs

Learn:

- Convolutions
- Feature maps
- Filters
- Pooling
- Why CNNs work on images

### Transfer Learning

Understand:

- Pretrained models
- Fine tuning
- Frozen layers
- Domain shift

### Evaluation

Learn:

- Precision
- Recall
- IoU
- Confidence
- False positives
- False negatives

---

## Practice Project

Train a very small image classifier in PyTorch.

The goal is simply to understand:

- datasets
- dataloaders
- training loops
- validation

---

## Success Criteria

Understand the difference between:

- inference,
- training,
- fine tuning.

---

# Module 5 — Object Detection (YOLO)

## Goal

Detect objects from Paesano's camera.

## Learn

### Detection Concepts

- Bounding boxes
- Object confidence
- Class confidence
- IoU
- Non-Maximum Suppression

### YOLO

Learn:

- Running pretrained models
- Live inference
- Model sizes
- FPS vs accuracy
- Confidence thresholds

### Domain Shift

Your robot camera is only ~4 inches above the ground.

Understand why that viewpoint is different from standard datasets.

### Fine Tuning

Only later.

Learn:

- Dataset creation
- Bounding-box annotation
- Data augmentation
- Retraining

---

## Practice Project

1. Run YOLO on images.
2. Run YOLO on recorded Paesano footage.
3. Run YOLO live on the robot.

---

## Success Criteria

You understand why detections succeed or fail.

---

# Module 6 — RGB-D Vision

## Goal

Convert YOLO detections into map coordinates.

## Learn

### Camera Model

- Intrinsics
- Camera matrix
- Optical frame
- Pixel coordinates

### Depth Images

Learn:

- Depth units
- Invalid depth
- RGB-depth alignment
- Noise

### Back Projection

Understand:

```text
Pixel

+

Depth

↓

3D Point
```

### Coordinate Frames

Learn:

```text
camera

↓

base_link

↓

odom

↓

map
```

### Filtering

Learn:

- Median depth
- Outlier rejection
- Temporal averaging

---

## Practice Project

Detect one object.

Project it into:

```text
Map Frame
```

Visualize it in RViz.

---

## Success Criteria

The same object stays at roughly the same map position as the robot moves.

---

# Module 7 — Semantic Mapping

## Goal

Combine geometry with ML.

Current Paesano map:

```text
Obstacle

Free Space

Robot Pose
```

Desired map:

```text
Kitchen

Bedroom

Hallway

Chair

Table

Refrigerator
```

## Learn

### Persistent Objects

Understand:

- Data association
- Landmark persistence
- Duplicate observations

### Region Association

Assign every object to a geometric region.

### Evidence Accumulation

Instead of:

```text
Saw chair once
```

Use:

```text
Observed chair 37 times
Confidence = 0.96
```

### Room Classification

Start with simple scoring.

Example:

Kitchen

- Refrigerator
- Sink
- Microwave

↓

Kitchen

Bedroom

- Bed
- Chair

↓

Bedroom

---

## Practice Project

Create fake regions.

Feed fake object detections into them.

Automatically label regions.

---

## Success Criteria

Understand how:

```text
YOLO Detection

↓

Persistent Object

↓

Region

↓

Semantic Label
```

works.

---

# Module 8 — Optional Advanced ML

Only after everything else works.

## Fine Tune YOLO

Train on robot-collected images.

## Train a Room Classifier

Input:

```text
Object counts
```

Output:

```text
Kitchen

Bedroom

Office
```

## CLIP

Investigate:

- Open-vocabulary recognition
- Scene understanding
- Text-guided navigation

---

# Recommended Learning Order

## Stage A — Geometry

1. Binary image processing
2. Morphology
3. Connected components
4. Distance transform
5. Region growing
6. Region features
7. Skeletons

---

## Stage B — Topology

8. Region adjacency graphs
9. Topological planning
10. Named-place navigation

---

## Stage C — Machine Learning

11. Supervised learning
12. CNN fundamentals
13. YOLO inference
14. Detection evaluation

---

## Stage D — Vision

15. Camera intrinsics
16. Depth projection
17. Camera-to-map transforms
18. RGB-D localization

---

## Stage E — Semantic Mapping

19. Persistent object landmarks
20. Object-to-region association
21. Semantic room classification
22. Named-place navigation

---

# What NOT to Study

Don't spend weeks relearning:

- ROS 2
- SLAM
- EKFs
- A*
- Path tracking
- Motor control
- General robotics

Those are already strengths of Paesano.

Focus your learning entirely on the perception and semantic layer you're adding.

---

# Final Goal

At the end of this project, Paesano should be able to:

- Automatically segment rooms and hallways from an occupancy grid.
- Build a topological map of an indoor environment.
- Detect household objects using YOLO.
- Localize those objects in the map frame using RGB-D vision.
- Associate objects with rooms.
- Infer semantic room labels.
- Navigate to destinations using names like:

```text
"Go to the kitchen."

"Go to the bedroom."

"Go to the office."
```

instead of only metric `(x, y)` goals.
