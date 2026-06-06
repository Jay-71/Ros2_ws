<p align="center">
  <h1 align="center">🤖 MINI R1 — Autonomous Campus Navigation Robot</h1>
  <p align="center">
    <b>AI-Powered Sign Detection &amp; Reactive Navigation in Gazebo Ignition</b>
    <br/>
    ROS 2 · Gazebo Harmonic · YOLOv8 · OpenCV
  </p>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS_2-Humble-blue?style=for-the-badge&logo=ros" alt="ROS 2 Humble"/>
  <img src="https://img.shields.io/badge/Gazebo-Ignition-orange?style=for-the-badge" alt="Gazebo Ignition"/>
  <img src="https://img.shields.io/badge/YOLOv8-Ultralytics-purple?style=for-the-badge" alt="YOLOv8"/>
  <img src="https://img.shields.io/badge/Python-3.10+-green?style=for-the-badge&logo=python" alt="Python 3.10+"/>
</p>

---

## 📋 Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [System Architecture](#system-architecture)
- [Repository Structure](#repository-structure)
- [Robot Specifications](#robot-specifications)
- [Sensor Suite](#sensor-suite)
- [Autonomous Navigation Pipeline](#autonomous-navigation-pipeline)
- [AI Sign Detection](#ai-sign-detection)
- [ROS 2 Topic Architecture](#ros-2-topic-architecture)
- [Prerequisites](#prerequisites)
- [Installation & Setup](#installation--setup)
- [Running the Project](#running-the-project)
- [Troubleshooting](#troubleshooting)
- [Authors](#authors)

---

## Overview

**MINI R1** is a four-wheeled differential-drive robot designed for the **Campus Challenge** — an autonomous navigation competition where the robot must traverse a structured campus environment, interpret directional signs (LEFT, RIGHT, FORWARD, STOP, ROTATE, GOAL) using real-time AI inference, and reach the goal without any human intervention.

The project is built entirely on the **ROS 2** ecosystem and simulated in **Gazebo Ignition**. A custom-trained **YOLOv8** model provides vision-based decision making, while a **Vector Field Histogram (VFH)** inspired reactive planner handles obstacle avoidance and path selection.

---

## Key Features

| Feature | Description |
|---|---|
| **Custom URDF Robot** | CAD-designed 4-wheel differential-drive chassis with STL meshes |
| **Multi-Sensor Fusion** | 360° GPU LiDAR, RGB Camera (640×480 @ 30 Hz), and IMU (100 Hz) |
| **YOLOv8 Sign Detection** | Custom-trained model recognizing 6 sign classes at 85%+ confidence |
| **Reactive Navigation** | VFH-inspired obstacle avoidance with dynamic sector scoring |
| **Breadcrumb Anti-Loop** | GPS-timestamped breadcrumb trail prevents the robot from revisiting areas |
| **Virtual Stop Walls** | Detected STOP signs create temporary virtual obstacles in the cost map |
| **ArUco Marker Detection** | Supplementary localization via ArUco marker scanning (IDs 24–27) |
| **Real-Time HUD** | Split-screen Command Center with live LiDAR + annotated camera feed |
| **Gazebo–ROS Bridge** | Fully configured `ros_gz_bridge` for seamless Ignition ↔ ROS 2 communication |

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        GAZEBO IGNITION                              │
│  ┌──────────────┐  ┌──────────┐  ┌──────────┐  ┌──────────────┐   │
│  │  DiffDrive    │  │  LiDAR   │  │  Camera  │  │     IMU      │   │
│  │  Plugin       │  │  Sensor  │  │  Sensor  │  │    Sensor    │   │
│  └──────┬───────┘  └────┬─────┘  └────┬─────┘  └──────┬───────┘   │
│         │               │             │                │           │
└─────────┼───────────────┼─────────────┼────────────────┼───────────┘
          │               │             │                │
     ┌────▼───────────────▼─────────────▼────────────────▼────┐
     │                  ros_gz_bridge                          │
     └────┬───────────────┬─────────────┬────────────────┬────┘
          │               │             │                │
     /cmd_vel_stamped  /r1_mini/   /r1_mini/camera/  /r1_mini/
          │            lidar        image_raw          imu
          │               │             │                │
     ┌────▼───────────────▼─────────────▼────────────────▼────┐
     │              reactive_nav.py (AutonomousCANBot)         │
     │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
     │  │   VFH-based  │  │   YOLOv8     │  │  Breadcrumb  │  │
     │  │   Obstacle   │  │   Sign       │  │  Anti-Loop   │  │
     │  │   Avoidance  │  │   Detector   │  │  System      │  │
     │  └──────────────┘  └──────────────┘  └──────────────┘  │
     └─────────────────────────────────────────────────────────┘
```

---

## Repository Structure

The project is organized into **two ROS 2 packages**:

```
ros2_ws/src/
├── mini_r1_v1_description/         # Robot Description Package
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── urdf/
│   │   ├── mini_r1.urdf.xacro      # Top-level robot assembly
│   │   ├── robot_core.xacro         # Chassis + 4 wheel links/joints
│   │   ├── sensors.xacro            # Camera + IMU links/joints & Gazebo sensors
│   │   ├── lidar.xacro             # LiDAR link/joint & gpu_lidar sensor config
│   │   └── gazebo_control.xacro    # DiffDrive & JointStatePublisher plugins
│   ├── meshes/                      # CAD-exported STL files
│   │   ├── base_link.STL
│   │   ├── TR_LF.STL / TR_LR.STL / TR_RF.STL / TR_RR.STL   (wheels)
│   │   ├── CAM.STL / IMU.STL / LIDAR.STL                    (sensors)
│   ├── materials/textures/          # Visual textures for Gazebo rendering
│   ├── config/
│   │   └── joint_names_MINI_R1_v1.yaml
│   ├── worlds/
│   │   ├── campus_challenge.sdf     # Primary competition world
│   │   ├── warehouse.sdf           # Alternate test environment
│   │   └── empty.sdf               # Blank world for debugging
│   └── launch/
│       └── rsp.launch.py            # Robot State Publisher launcher
│
├── mini_r1_v1_gz/                   # Simulation & Autonomy Package
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/
│   │   └── ros_gz_bridge.yaml       # Full Gazebo ↔ ROS 2 bridge mapping
│   ├── launch/
│   │   └── sim.launch.py            # Main simulation launcher
│   ├── scripts/
│   │   ├── reactive_nav.py          # Core autonomous navigation node
│   │   └── aruco_detector.py        # ArUco marker detection node
│   └── runs/detect/
│       └── canbot_signs3/weights/   # Trained YOLOv8 model weights (best.pt)
```

---

## Robot Specifications

| Parameter | Value |
|---|---|
| **Name** | MINI_R1 |
| **Drive Type** | 4-Wheel Differential Drive |
| **Chassis Mass** | 1.173 kg |
| **Wheel Radius** | 0.065 m |
| **Wheel Separation** | 0.2885 m |
| **Max Linear Speed** | 0.6 m/s |
| **Max Angular Speed** | 1.2 rad/s |
| **Max Wheel Torque** | 50 Nm |
| **Max Linear Acceleration** | 1.5 m/s² |

### Kinematic Layout

```
            FRONT
     ┌──────────────────┐
     │   TR_LF    TR_RF │   ← Front wheels
     │                  │
     │    [base_link]   │
     │   [LIDAR] [IMU]  │
     │      [CAM]→      │   ← Camera faces forward
     │                  │
     │   TR_LR    TR_RR │   ← Rear wheels
     └──────────────────┘
            REAR
```

---

## Sensor Suite

### 1. GPU LiDAR
| Parameter | Value |
|---|---|
| Type | `gpu_lidar` |
| Samples | 360 |
| FOV | 360° (−π to +π) |
| Range | 0.12 m – 30.0 m |
| Update Rate | 10 Hz |
| ROS Topic | `/r1_mini/lidar` |
| Message Type | `sensor_msgs/msg/LaserScan` |

### 2. RGB Camera
| Parameter | Value |
|---|---|
| Resolution | 640 × 480 |
| Horizontal FOV | 2.1 rad (~120°) |
| Clip Range | 0.05 m – 8.0 m |
| Update Rate | 30 Hz |
| ROS Topic | `/r1_mini/camera/image_raw` |
| Message Type | `sensor_msgs/msg/Image` |

### 3. IMU
| Parameter | Value |
|---|---|
| Update Rate | 100 Hz |
| Angular Velocity Noise (σ) | 2×10⁻⁴ rad/s |
| Linear Acceleration Noise (σ) | 1.7×10⁻² m/s² |
| ROS Topic | `/r1_mini/imu` |
| Message Type | `sensor_msgs/msg/Imu` |

---

## Autonomous Navigation Pipeline

The `reactive_nav.py` node (`AutonomousCANBot`) implements a fully autonomous navigation stack consisting of the following subsystems:

### 1. Vector Field Histogram (VFH) Obstacle Avoidance
- LiDAR data is divided into **48 polar sectors** spanning the front 240° FOV.
- Each sector receives a **Polar Obstacle Density (POD)** score based on proximity.
- A **dynamic safe distance** is applied — wider margins ahead, tighter on sides.
- Sector dilation ensures the robot accounts for its own physical width.
- Valleys (contiguous groups of low-POD sectors) are identified, and the optimal one is selected using a cost function that balances **heading preference** and **steering smoothness**.

### 2. Breadcrumb Anti-Loop System
- Odometry-based breadcrumbs are dropped every **0.4 m** of travel.
- Breadcrumbs older than **45 seconds** are pruned automatically.
- Recent breadcrumbs inject **repulsive cost** into nearby LiDAR sectors, biasing the robot to explore unvisited areas.
- LiDAR occlusion filtering ensures breadcrumbs hidden behind walls do not create phantom repulsion.

### 3. Sign-Driven Decision Making
- Vision detections are categorized as **PENDING** (seen far away) or **ARMED** (within proximity threshold).
- **LEFT**: Biases the target sector to 40/48 (sharp left) for 4.5s, with a 30s cooldown.
- **RIGHT**: Biases the target sector to 8/48 (sharp right) for 4.5s, with a 30s cooldown.
- **FORWARD**: Locks sector to center (24/48) for 1.5s.
- **ROTATE**: Executes a blind 360° spin for 2s, then searches for the sign to re-acquire heading.
- **STOP**: Halts the robot for 2.5s and erects a virtual wall ahead for 20s.
- **GOAL**: Permanently halts the robot — mission accomplished.

### 4. GPS-Based Sign Memory
- Each detected sign is recorded with its `(x, y, label)` in memory.
- Signs within **6.0 m** of a previously seen sign of the same class are suppressed, preventing redundant reactions.

---

## AI Sign Detection

### Model Architecture
- **Framework**: Ultralytics YOLOv8
- **Inference Resolution**: 320 × 320 px
- **Confidence Threshold**: 85% (general), 98% (STOP signs — to avoid false positives)
- **Device**: GPU (`device=0`)

### Recognized Classes

| Class | Action | Color Code (BGR) |
|---|---|---|
| `LEFT` | Turn left at next junction | 🟠 `(255, 165, 0)` |
| `RIGHT` | Turn right at next junction | 🔵 `(30, 144, 255)` |
| `FORWARD` | Continue straight | 🟢 `(50, 205, 50)` |
| `STOP` | Halt for 2.5s, place virtual wall | 🔴 `(0, 0, 255)` |
| `GOAL` | Mission complete — permanent halt | 🟡 `(255, 215, 0)` |
| `ROTATE` | Execute 360° scan rotation | 🟣 `(238, 130, 238)` |

### Filtering & Validation
- **Aspect Ratio Check**: Detections with aspect ratios outside `[0.50, 2.00]` are discarded.
- **Proximity Ratio**: Sign height relative to the frame height must exceed `0.22` (directional) or `0.26` (STOP) to trigger execution.
- **Per-Class Cooldowns**: 30-second cooldown per sign class after execution to prevent re-triggering.

---

## ROS 2 Topic Architecture

The `ros_gz_bridge.yaml` establishes full bidirectional communication:

| ROS 2 Topic | Gazebo Topic | Type | Direction |
|---|---|---|---|
| `/clock` | — | `rosgraph_msgs/Clock` | GZ → ROS |
| `/cmd_vel_stamped` | `/cmd_vel` | `geometry_msgs/TwistStamped` | ROS → GZ |
| `/cmd_vel` | `/cmd_vel` | `geometry_msgs/Twist` | ROS → GZ |
| `/r1_mini/odom` | `/r1_mini/odom` | `nav_msgs/Odometry` | GZ → ROS |
| `/joint_states` | `/r1_mini/joint_states` | `sensor_msgs/JointState` | GZ → ROS |
| `/tf` | `/r1_mini/odom_tf` | `tf2_msgs/TFMessage` | GZ → ROS |
| `/r1_mini/lidar` | `/r1_mini/lidar` | `sensor_msgs/LaserScan` | GZ → ROS |
| `/r1_mini/imu` | `/r1_mini/imu` | `sensor_msgs/Imu` | GZ → ROS |
| `/r1_mini/camera/image_raw` | `/r1_mini/camera` | `sensor_msgs/Image` | GZ → ROS |
| `/r1_mini/camera/camera_info` | `/r1_mini/camera_info` | `sensor_msgs/CameraInfo` | GZ → ROS |

---

## Prerequisites

| Dependency | Version | Purpose |
|---|---|---|
| **Ubuntu** | 22.04 LTS | Host operating system |
| **ROS 2** | Humble Hawksbill | Robot middleware framework |
| **Gazebo** | Ignition / Harmonic | Physics simulation |
| **ros_gz_sim** | Latest | Gazebo–ROS 2 integration |
| **ros_gz_bridge** | Latest | Topic bridging between Gazebo and ROS 2 |
| **twist_stamper** | Latest | Converts `Twist` → `TwistStamped` |
| **Python** | 3.10+ | Scripting runtime |
| **ultralytics** | Latest | YOLOv8 inference engine |
| **OpenCV** | 4.x+ | Image processing and ArUco detection |
| **NumPy** | Latest | Numerical operations |
| **cv_bridge** | ROS 2 pkg | ROS Image ↔ OpenCV conversion |

---

## Installation & Setup

### 1. Clone the Repository

```bash
cd ~/ros2_ws/src
git clone <repository-url>
```

### 2. Install System Dependencies

```bash
# ROS 2 packages
sudo apt update
sudo apt install -y \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-twist-stamper \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-cv-bridge \
  ros-humble-teleop-twist-keyboard
```

### 3. Install Python Dependencies

```bash
pip install ultralytics opencv-python numpy
```

### 4. Build the Workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

### 5. Source the Workspace

```bash
source ~/ros2_ws/install/setup.bash
```

> **Tip**: Add the above line to your `~/.bashrc` to source automatically on every new terminal.

---

## Running the Project

### Step 1 — Launch the Gazebo Simulation

This starts Gazebo Ignition with the `campus_challenge.sdf` world, publishes the robot description, spawns the MINI R1 entity, and launches the `ros_gz_bridge`.

```bash
# Terminal 1
source ~/ros2_ws/install/setup.bash
ros2 launch mini_r1_v1_gz sim.launch.py
```

> Wait for Gazebo to fully load and the robot to appear in the environment.

### Step 2 — Launch the Autonomous Navigation Node

This starts the `AutonomousCANBot` node, which subscribes to the LiDAR, Camera, and Odometry topics, runs YOLOv8 inference on the camera feed, and publishes velocity commands.

```bash
# Terminal 2
source ~/ros2_ws/install/setup.bash
ros2 run mini_r1_v1_gz reactive_nav.py
```

A **CANBot Command Center** window will appear showing:
- **Left Panel**: Top-down LiDAR point cloud with breadcrumb trails, virtual walls, and the chosen heading vector.
- **Right Panel**: Annotated camera feed with bounding boxes and sign labels overlaid.
- **Status Bar**: Current behavioral state (EXPLORING, EXECUTING TURN, BLOCKED, GOAL REACHED, etc.)

### (Optional) Step 3 — Launch the ArUco Detector

For ArUco marker-based localization tasks:

```bash
# Terminal 3
source ~/ros2_ws/install/setup.bash
ros2 run mini_r1_v1_gz aruco_detector.py
```

### (Optional) Manual Teleoperation

To manually drive the robot instead of using the autonomous pipeline:

```bash
# Terminal 2 (instead of reactive_nav)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Custom World

To launch with a different world file:

```bash
ros2 launch mini_r1_v1_gz sim.launch.py world:=/absolute/path/to/your/world.sdf
```

---

## Troubleshooting

| Issue | Solution |
|---|---|
| Gazebo fails to find the robot model | Ensure `IGN_GAZEBO_RESOURCE_PATH` is set. The launch file handles this automatically. Verify with `echo $IGN_GAZEBO_RESOURCE_PATH`. |
| Camera topic not publishing | Check `ros2 topic list` for `/r1_mini/camera/image_raw`. Ensure the bridge is running. |
| YOLOv8 model not loading | Verify the `best.pt` weights exist at `runs/detect/canbot_signs3/weights/best.pt` inside the `mini_r1_v1_gz` package. |
| `twist_stamper` not found | Install with `sudo apt install ros-humble-twist-stamper`. |
| Robot not moving | Confirm `/cmd_vel` or `/cmd_vel_stamped` is being published. Use `ros2 topic echo /cmd_vel` to debug. |
| OpenCV GUI error | If running headless, set `export QT_LOGGING_RULES='qt.qpa.fonts.warning=false'`. The script does this automatically. |

