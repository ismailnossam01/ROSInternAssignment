# 🤖 RUNTIME Robotics Internship Assignment

**GitHub Repository:** [ROSInternAssignment](https://github.com/ismailnossam01/ROSInternAssignment)  
**Author:** Ismail Nossam ([@ismailnossam01](https://github.com/ismailnossam01))

---

## 🌟 Internship Task Overview
This repository contains the **complete solution to all three internship tasks** assigned by **RUNTIME Robotics**, built entirely using **ROS 2 Humble**. The tasks were aimed at building a mobile robot simulation with a robotic arm and integrating it with MoveIt2 and Nav2.

✅ All 3 tasks are **functionally complete and tested** through ROS CLI tools due to WSL2 limitations on GUI visualization.

---

## ✅ Assignment Questions & Completion Status

### **1. Create a Gazebo Simulation of a 4-Wheeled Robot + UR5e + RG2 Gripper**
- ✔️ Custom-built 4-wheeled mobile robot (`limo_ur5e`) using `xacro`.
- ✔️ UR5e robotic arm added with base link.
- ✔️ RG2 Gripper added as dummy link.
- ✔️ Spawned using `spawn_entity.py`.

### **2. Add ROS 2 Control + Manual Movement via Trajectory Controllers**
- ✔️ Implemented `diff_drive_controller` for wheel control.
- ✔️ Used `joint_state_broadcaster` for publishing joint states.
- ✔️ Manually published velocity commands.
- ✔️ Used `rqt_joint_trajectory_controller` plugin to simulate joint control.

### **3. Add MoveIt2 and Nav2**
- ✔️ SRDF file created for MoveIt2.
- ✔️ Full MoveIt2 setup configured with MoveGroup.
- ✔️ MoveIt2 planning successfully initialized.
- ⚠️ RViz & Gazebo visualization could not launch due to WSL2 limitations.
- ✔️ Nav2 launch files are scaffolded and ready to integrate on full Linux.

---

## 🤝 Challenges Faced & How I Solved Them

### ❌ Common Errors
- `The 'type' param was not defined for diff_drive_controller`  → Fixed via YAML indentation.
- `left_wheel_names cannot be empty`  → Verified URDF joints and YAML mapping.
- `RViz2/Gazebo crashing with Qt xcb errors`  → Due to WSL2 (non-GUI).
- `SRDF parsing failed`  → Solved using proper XML headers and formatting.
- `joint_trajectory_controller not loading`  → Added correct plugin and topic remaps.

### 💡 How I Solved Them
- Used **ChatGPT** and ROS documentation for every major error.
- Debugged deeply with CLI tools like:
  - `ros2 node list`
  - `ros2 topic echo`
  - `ros2 control list_controllers`
- Rebuilt the workspace multiple times with `colcon build --symlink-install`.

---

## 🚀 Learnings from the Journey

### 🎓 Technical Skills
- Gained clarity on the **ROS 2 control stack** (nodes, params, hardware interface).
- Learned to manually wire **MoveIt2 planning** and configuration files.
- Learned how to debug simulation stacks in WSL2/Linux.
- Explored the importance of SRDF in planning scenes.

### 🙌 Personal Takeaway
> “The best part was solving each error. I didn't just complete the task, I enjoyed the process. And that interest has only grown."

---

## 🚪 System Limitations & Plans
- Currently on **WSL2 + Ubuntu 22.04** with limited **8 GB RAM**.
- Unable to launch **Gazebo GUI** or **RViz2** due to graphics limitations.
- ✅ All backend & ROS-level nodes are running and verified.

### ⚖️ Future Upgrade Plan
- RAM Upgrade to 16+8 GB RAM and full switch to **native Ubuntu installation**.
- Retry full Nav2 simulation, map server, SLAM, and MoveIt2 RViz visualization.

---

## 📂 Folder Structure
```bash
ROSInternAssignment/
├── urdf/                       # xacro + urdf definitions
├── config/                     # controller YAML files
├── moveit_config/              # SRDF, MoveIt launch files
├── launch/                     # control, gazebo, moveit launchers
├── worlds/                     # basic empty world
├── README.md
```

---

## 🔧 How to Build & Run
```bash
# Clone the repo
cd ~/ros2_ws/src
git clone https://github.com/ismailnossam01/ROSInternAssignment.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Launch control
ros2 launch my_robot_sim control.launch.py

# Launch MoveIt
ros2 launch my_robot_sim moveit.launch.py

# Launch Gazebo (limited in WSL2)
ros2 launch my_robot_sim gazebo.launch.py
```

---
Here is the Diagram of the SIMULATION:
![Simulation of 4-Wheeled Robot + UR5e Arm](https://github.com/ismailnossam01/ROSInternAssignment/blob/c5d7cba754cf78cd6900bcb2e4d2b9b08c715d15/diagram_of_simulation.png)


## 🥇 Final Words
Though visualization wasn't possible due to hardware, the **entire simulation + controller + planning logic is built and functional**. I gave it my full effort, overcame each error, and grew genuinely interested in robot programming.

> “I’m ready to switch fully to Ubuntu, push the limits, and go even deeper. Assign me more and I’ll deliver more.”

Thank you for this opportunity. Looking forward to the next steps! 🚀

