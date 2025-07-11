# Week 4A - Robotic Arm Assignment (Robotics & Controls)

This repository contains my solutions for the **Week 4A Assignment** as part of the course. The project simulates a 2-link robotic arm in ROS 2 using Python, URDF, RViz2, and the Joint State Publisher GUI.

## Code Mapping to Questions

| Question | Code File                        | Description |
|----------|----------------------------------|-------------|
| Q1       | `forward_kinematics_node.py`     | Computes the 2D end-effector position ((x, y)) from θ₁ and θ₂ and publishes it on `/end_effector_position`. |
| Q3       | `inverse_kinematics_node.py`     | Takes current end-effector position and moves in user-defined direction. Publishes target joint angles to `/joint_angles_goal`. |
| Bonus    | `forward_kinematics_3d_node.py`  | Computes the **3D end-effector position** ((x, y, z)) using θ₀ (base yaw), θ₁, and θ₂, then publishes it. |

## How to Build and Run

bash
cd ~/ros2_week4
colcon build
source install/setup.bash


