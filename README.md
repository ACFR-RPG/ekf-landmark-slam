# MTRX5700 Assignment 3 — Landmark-Based EKF SLAM on TurtleBot3

This workspace implements landmark-based Simultaneous Localisation and Mapping (SLAM) using an Extended Kalman Filter (EKF) on a TurtleBot3 robot. The system can run against a Gazebo simulation or on the real robot.

## Repository Structure

```
assignment_3/
├── turtlebot_landmark_slam/   # Main ROS 2 package — EKF SLAM pipeline
├── landmarks_msg/             # Custom ROS 2 message definitions
└── third_parties/             # External TurtleBot3 packages (read-only)
    ├── turtlebot3/
    ├── turtlebot3_msgs/
    ├── turtlebot3_simulations/
    └── DynamixelSDK/
```

## Main Packages

### `turtlebot_landmark_slam`

The core package. It implements the full EKF SLAM pipeline — reading sensor data, running the filter, and publishing the estimated pose and map.

**Source modules (`src/turtlebot_landmark_slam/`):**

| Module | Description |
|--------|-------------|
| `ekf.py` | **Student task.** The `ExtendedKalmanFilter` class. `predict()` and `update()` are to be implemented. Maintains the joint state vector `[x, y, θ, l1x, l1y, ...]` and its covariance. |
| `pipeline.py` | Wires the EKF to ROS 2. Handles control and landmark callbacks, calls `ekf.predict()` / `ekf.update()`, and publishes the estimated odometry and landmark map as a `MarkerArray`. |
| `dataprovider.py` | Converts raw ROS messages into typed `ControlMeasurement` / `LandmarkMeasurement` objects. `SimulationDataProvider` adds Gaussian noise to simulate real-world odometry error; `OnlineDataProvider` is used with the physical robot. |
| `landmarks_circle_detector.py` | Detects cylindrical landmarks in 2D laser scan data. Clusters scan points, fits circles using algebraic least squares + Levenberg-Marquardt refinement, and propagates fit covariance. Supports Cartesian and polar output. |
| `types.py` | Data classes: `LandmarkMeasurement` (x, y, label, 2×2 covariance) and `ControlMeasurement` (dx, dy, dθ, 3×3 covariance). |
| `utils.py` | Geometry helpers used inside the EKF: `Relative2AbsolutePose`, `Relative2AbsoluteXY`, `Absolute2RelativeXY`, `pi2pi`, and `RelativeLandmarkPositions`. |

**Scripts (`scripts/`):**

| Script | Description |
|--------|-------------|
| `ekf_pipeline_node.py` | ROS 2 node entry point. Creates an `EkfPipelineNode` that instantiates `Pipeline` with an `ExtendedKalmanFilter`. |
| `odom_to_control_republisher.py` | Simulation helper. Republishes the twist component of `/odom` as a `Twist` control message consumed by the EKF pipeline. Publishes at the Gazebo odometry rate (~50 Hz). |
| `landmark_publisher_sim.py` | Simulation helper. Reads the four cylinder obstacle positions from the DQN Stage 2 world SDF, transforms them into the robot body frame using `/odom`, and publishes them as `LandmarksMsg` on `/landmarks` at 2 Hz. Measurement noise is controlled by the `std_dev_landmark_x` and `std_dev_landmark_y` node parameters (default 0.01 m², ~0.1 m std dev). |
| `map_writer.py` | Subscribes to `/ekf/map` (a `MarkerArray`) and writes the estimated landmark positions to `map_slam.txt` in `POINT2D <id> <x> <y>` format. |
| `evaluate_map.py` | Off-line evaluation tool. Compares `map_slam.txt` against a ground-truth file and reports relative landmark position error. |

**Launch files (`launch/`):**

| File | Description |
|------|-------------|
| `simulation.launch.py` | Starts Gazebo with the TurtleBot3 DQN Stage 2 world, `odom_to_control_republisher`, and `landmark_publisher_sim`. Run this first when testing in simulation. |
| `ekf_pipeline.launch.py` | Starts the EKF pipeline node. Accepts an `is_real` argument (`true` for physical robot, `false` (default) for simulation). Topic remappings differ between modes. |

### `landmarks_msg`

A minimal ROS 2 interface package that defines the custom messages used to carry landmark detections between nodes.

| Message | Fields |
|---------|--------|
| `LandmarkMsg` | `uint64 label`, `float32 x`, `float32 y`, `float32 s_x`, `float32 s_y` — a single landmark detection with position and measurement standard deviations. |
| `LandmarksMsg` | `LandmarkMsg[] landmarks` — an array of detections published each scan. |

## Third-Party Packages

The `third_parties/` directory contains upstream TurtleBot3 packages included for convenience. These are not modified as part of the assignment:

- **`turtlebot3`** — core drivers, URDF models, and bringup for the TurtleBot3 platform.
- **`turtlebot3_simulations`** — Gazebo world and model files, including the Stage 2 environment used by the simulation launch file.
- **`turtlebot3_msgs`** — TurtleBot3-specific ROS 2 message and service definitions.
- **`DynamixelSDK`** — Low-level SDK for the Dynamixel servo motors used in TurtleBot3's drive system.

## Data Flow

### Simulation (`is_real:=false`)

```
Gazebo
  ├─ /odom (Odometry, ~50 Hz) ──► odom_to_control_republisher ──► /control (Twist, ~50 Hz) ──► EKF.predict()
  └─ /odom (Odometry, ~50 Hz) ──► landmark_publisher_sim ──────► /landmarks (LandmarksMsg, 2 Hz) ──► EKF.update()
           (robot pose)             (transforms SDF cylinder
                                     positions to robot frame)
```

### Real Robot (`is_real:=true`)

```
TurtleBot3
  ├─ /cmd_vel (Twist) ───────────────────────────────────────────► /control ──► EKF.predict()
  └─ /scan (LaserScan) ──► landmarks_circle_detector ────────────► /landmarks ──► EKF.update()
```

### Common output (both modes)

```
EkfPipelineNode
  ├─ /ekf/odom  (Odometry)    — estimated robot pose + covariance
  └─ /ekf/map   (MarkerArray) — estimated landmark positions
                                      │
                                map_writer.py → map_slam.txt
```

## Student Task

The only file requiring implementation is `ekf.py`:

- **`predict(control_measurement)`** — EKF prediction step. Use `utils.Relative2AbsolutePose` to compute the predicted pose and Jacobians `F` and `W`, then propagate the state mean and covariance.
- **`update(landmark_measurement, is_new)`** — EKF update step. For a new landmark, initialise its position with `utils.Relative2AbsoluteXY`. For a previously seen landmark, compute the expected measurement with `utils.Absolute2RelativeXY`, form the innovation, innovation covariance `S`, Kalman gain `K`, and apply the update to the full joint state.

## Running

**Simulation:**
```bash
# Terminal 1 — start Gazebo
ros2 launch turtlebot_landmark_slam simulation.launch.py

# Terminal 2 — start EKF
ros2 launch turtlebot_landmark_slam ekf_pipeline.launch.py is_real:=false
```

**Real robot:**
```bash
ros2 launch turtlebot_landmark_slam ekf_pipeline.launch.py is_real:=true
```

**Move the robot (teleoperation):**
```bash
# Terminal 3 — use keyboard to drive, Ctrl+C to stop
export TURTLEBOT3_MODEL=burger; ros2 run turtlebot3_teleop teleop_keyboard
```

**Save the map:**
```bash
ros2 run turtlebot_landmark_slam map_writer.py
```

**Evaluate against ground truth:**
```bash
python3 scripts/evaluate_map.py --solution map_slam.txt --gt <ground_truth_file>.txt
```
