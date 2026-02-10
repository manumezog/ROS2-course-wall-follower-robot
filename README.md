# Wall Follower ROS2 Package

This repository contains the **wall_follower** ROS2 package, developed as the final project for my **ROS2 Certification** from [TheConstruct.AI](https://www.theconstruct.ai/).

## Certification Details

- **Course**: [ROS2 Basics in 5 Days (Python)](https://app.theconstruct.ai/courses/ros2-basics-in-5-days-v2-python-268/)
- **Institution**: TheConstruct.AI
- **Status**: Final Project - **Phase 1 (ROS2 Topics) Completed**

## Project Scope

As part of this certification, I have:

1.  **Simulated the robot in Gazebo**: Developed and tested the wall-following logic in a controlled virtual environment.
2.  **Remote Real Robot Execution**: Deployed and validated the code on a real robot located at TheConstruct labs using their remote lab environment.

## Scripts & Control Logic

This package includes two versions of the wall-following algorithm:

- **`wall_following.py` (Main Version)**:
  - **Status**: Fully tested and verified on both **Gazebo simulation** and the **real robot**.
  - **Logic**: Implements **proportional control (P-controller)** actions to ensure smooth and stable robot movement along walls. It adjusts speed and steering based on the distance error, providing a much more robust performance in real-world conditions.
- **`wall_following_backup.py` (Legacy Version)**:
  - **Status**: Tested only in **Gazebo simulation**.
  - **Logic**: Uses a simpler "bang-bang" or discrete logic. It does not include proportional actions, making it less smooth but easier to debug during the initial development phases.

## 🛠️ Hardware/Software Stack

To ensure reproducibility, this project was developed and tested in the following environment:

- **Robot/Simulation**: Custom Mars Rover simulation and actual TurtleBot-based platform (via TheConstruct labs).
- **OS**: Ubuntu 24.04 (WSL2)
- **Middleware**: ROS2 Humble
- **Sensors**: LiDAR (LaserScan)
- **Language**: Python 3.10+

## 🧠 The Logic Flow (The "Why")

The core navigation logic is built on a **P-Controller** designed for stability and precision.

- **Proportional Gain ($K_p = 3$)**: Tuned to minimize distance error from the wall (Setpoint: 0.30m) without inducing oscillations.
- **Speed Profiling**: I opted for a **cruise speed of $0.1$ m/s** to prioritize sensor accuracy and data stability over raw speed. This ensures that the LiDAR measurements remain reliable even in tight corners or narrow passages.

## 🔄 Obstacle Recovery

One of the key features of this implementation is its robustness in high-complexity environments.

- **Stuck Detection**: Implemented a **timed watchdog** mechanism. If the frontal distance remains below 0.5m (or 0.35m depending on sensor noise) for more than **1.8s**, it triggers a deadlock resolution.
- **Recovery Maneuver**: The robot automatically executes a **Reverse-Pivot** maneuver:
  1.  **Phase 1**: Gentle reverse away from the obstruction.
  2.  **Phase 2**: Controlled pivot to re-orient the heading towards clear space.
  3.  **Phase 3**: Hand-off back to the P-Controller for normal navigation.

## Course Overview

ROS2 is the next generation of the Robot Operating System. This introductory course provides the essential foundations for working with ROS2, moving beyond basic "bells and whistles" to focus on core concepts used in professional robotics development.

### Learning Objectives

- **Package Management**: Creation and structuring of ROS2 packages.
- **Build System**: Management of the new **Colcon** universal building system.
- **Communication (Topics)**: Implementation of Topic Publishers and Subscribers in Python.
- **Launch System**: Utilizing the new Python-based launch system for complex node configurations.
- **Services**: Generation and implementation of Service servers and clients.
- **Interoperability**: Basic use of **ROS1-Bridge** to communicate between ROS1 and ROS2 systems.
- **Testing & Debugging**: Effective use of ROS2 debugging tools.
- **Advanced Concepts**: Understanding Callbacks, Multithreading, and ROS2 Actions.

## Course Summary & Progress

The following table tracks the progress through the certification curriculum:

| Section | Topic                       | Status                          |
| :------ | :-------------------------- | :------------------------------ |
| 1       | Introduction to the Course  | 100%                            |
| 2       | ROS2 Basic Concepts         | 100%                            |
| 3       | Understanding ROS2 Topics   | 92% (Phase 1 Project Completed) |
| 4       | Understanding ROS2 Services | 0%                              |
| 5       | Callbacks in ROS 2          | 0%                              |
| 6       | Multithreading              | 0%                              |
| 7       | Understanding ROS2 Actions  | 0%                              |
| 8       | Debugging Tools             | 0%                              |
| 9       | Final Recommendations       | -                               |

---

_This project is a work in progress as I continue through the ROS2 certification path._
