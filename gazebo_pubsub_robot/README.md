# ROS 2 Gazebo Pub-Sub Mobile Robot (WASD + Odometry)

## Project Overview

This project demonstrates a **complete ROS 2 publisher–subscriber control loop** using a **custom URDF mobile robot simulated in Gazebo**.

The robot:

* Is modeled using **URDF**
* Is simulated with **Gazebo physics**
* Uses **ROS 2 topics** for control and feedback
* Moves using **WASD keyboard input**
* Publishes and subscribes to **odometry (`/odom`)**

This is a **minimal but industry-correct mobile robotics architecture** that directly maps to real robots.

---

## System Architecture

```
Keyboard (W A S D)
        ↓
ROS 2 Teleop Node (Publisher)
        ↓   /cmd_vel
Gazebo Diff Drive Plugin (Subscriber)
        ↓
Physics Engine
        ↓
Robot Motion
        ↓
Odometry (/odom)
        ↑
ROS 2 Teleop Node (Subscriber)
```

---

## ROS Topics Used

| Topic      | Message Type        | Purpose                           |
| ---------- | ------------------- | --------------------------------- |
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands (publisher)     |
| `/odom`    | nav_msgs/Odometry   | Robot state feedback (subscriber) |

---

## Robot Description (URDF)

* **Base**: Box-shaped chassis (`base_link`)
* **Drive wheels**: Two cylindrical wheels with continuous joints
* **Caster wheel**: Fixed spherical caster for stability
* **Drive plugin**: `gazebo_ros_diff_drive`

The URDF includes:

* Correct wheel orientation
* Accurate wheel separation and diameter
* Inertial values for stable physics
* Low-friction caster configuration

This ensures **realistic and stable simulation behavior**.

---

## Prerequisites

* Ubuntu 22.04
* ROS 2 Humble
* Gazebo 11
* gazebo_ros packages installed

Quick verification:

```
echo $ROS_DISTRO
gazebo --version
```

---

## Build Instructions

From the workspace root:

```
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Always rebuild after modifying URDF, launch files, or Python nodes.

---

## Run Instructions

### Terminal 1 — Launch Gazebo and Spawn Robot

```
ros2 launch gazebo_pubsub_robot spawn_robot.launch.py
```

This:

* Starts Gazebo with ROS integration
* Loads the factory plugin
* Spawns the URDF robot
* Enables physics and diff-drive control

---

### Terminal 2 — Start Keyboard Teleoperation

```
ros2 run gazebo_pubsub_robot teleop_wasd
```

⚠️ Click inside this terminal so it captures keyboard input.

---

## Keyboard Controls

| Key         | Action           |
| ----------- | ---------------- |
| W           | Move forward     |
| S           | Move backward    |
| A           | Turn left        |
| D           | Turn right       |
| Release key | Stop             |
| Q           | Quit teleop node |

The robot moves **only while a key is held**, which matches real robot behavior.

---

## Odometry Feedback

The teleop node subscribes to `/odom` and prints:

* Position (x, y)
* Orientation (yaw in degrees)

Example output:

```
ODOM | x: 0.52  y: 0.01  yaw: 3.4°
```

This enables **closed-loop control** instead of blind motion.

---

## Why Odometry Is Important

* `/cmd_vel` expresses **intent**
* `/odom` reports **what actually happened**

Without odometry:

* Distance control is impossible
* Accurate rotation is impossible
* Autonomous behavior cannot be built

This project follows **closed-loop robotics principles**.

---

## Debugging Commands

Check velocity commands:

```
ros2 topic echo /cmd_vel
```

Check odometry:

```
ros2 topic echo /odom
```

Check Gazebo spawn service:

```
ros2 service list | grep spawn
```

---
