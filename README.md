# Panda Robot Pick-and-Place with MoveIt Task Constructor

**Course:** Robot Programming 

**Team Members:**
- Member 1: Chen Yuheng(System Modeling & Environment Setup)
- Member 2: Xiangzhang(Pick-and-Place Logic & Finite State Sequence)
*(Note: Please verify if the roles above match your actual contribution)*

## 1. Goal and Objectives
The objective of this project is to implement a robust pick-and-place pipeline for the Franka Emika Panda robot using **MoveIt Task Constructor (MTC)**.
The system identifies a cylinder object, plans a collision-free trajectory, picks it up, and places it at a designated target location.

## 2. Implemented Features
- **Finite State Machine (FSM):** Structured task stages (Open Hand -> Pick -> Place -> Home).
- **Inverse Kinematics (IK):** Custom grasp pose generation with orientation adjustments (90-degree rotation).
- **Hybrid Planning:** Combines `PipelinePlanner` (OMPL) for long-distance collision avoidance and `CartesianPath` for precision approach/retreat.
- **Visualization:** Integrated RViz visualization with task solution trails.

## 3. Technologies
- **OS:** Ubuntu 22.04 LTS (Jammy Jellyfish)
- **Middleware:** ROS 2 Humble Hawksbill
- **Library:** MoveIt 2, MoveIt Task Constructor
- **Language:** C++ (Core Logic), Python (Launch system)

## 4. How to Launch

### Prerequisites
Ensure you have ROS 2 and MoveIt 2 installed.

### Installation & Build
1. Create a workspace and clone the repository:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   # Clone this repository into src
   git clone <YOUR_GITHUB_LINK_HERE>

2.Install dependencies:
```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -y

3.Build the package:
```bash
colcon build --packages-select panda_pick_place
source install/setup.bash
   
  ### Run
  Step 1: Launch Environment & MoveIt
This launches the fake robot controller, RViz, and the MoveGroup capability:
```bash
ros2 launch panda_pick_place mtc_demo.launch.py

  Step 2:Execute Pick and Place Task
Open a new terminal, source the workspace, and run the logic node:
```bash
source install/setup.bash
ros2 launch panda_pick_place demo_pick_and_place.launch.py
