# Panda Robot Pick-and-Place with MoveIt Task Constructor

Course: Robot Programming 
Team Members:
- Member 1: Chen Yuheng (System Modeling & Environment)
- Member 2: Xiangzhang (Pick-and-Place Logic & FSM)

## 1. Goal and Objectives
The objective of this project is to implement a robust pick-and-place pipeline for the Franka Emika Panda robot using "MoveIt Task Constructor (MTC)".
The system identifies a cylinder object, plans a collision-free trajectory, picks it up, and places it at a designated target location.

## 2. Implemented Features
- Finite State Machine (FSM): Structured task stages (Open Hand -> Pick -> Place -> Home).
- Inverse Kinematics (IK): Custom grasp pose generation with orientation adjustments.
- Hybrid Planning: Combines `PipelinePlanner` (OMPL) for long-distance motion and `CartesianPath` for precision approach/retreat.
- Visualization: Integrated RViz visualization with trajectory trails.

## 3. Technologies
- OS: Ubuntu 22.04.3 LTS
- Middleware: ROS 2 Jazzy
- Library: MoveIt 2, MoveIt Task Constructor
- Language: C++ (Core Logic), Python (Launch system)

## 4. How to Launch

### Prerequisites
Ensure you have ROS 2 Jazzy and MoveIt 2 installed.

### Build
```bash
cd ~/ros2_ws
colcon build --packages-select mtc
source install/setup.bash

## Run
Step 1: Launch Environment & MoveIt
ros2 launch mtc_tutorial mtc_demo.launch.py

Step 2: Execute Pick and Place Task
ros2 launch mtc_tutorial demo_pick_and_place.launch.py
