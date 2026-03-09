# UAV Autonomy Project

This repository contains a UAV autonomy control stack built on top of **ArduPilot** and **MAVLink**.

The goal of this project is to develop an **autonomous drone capable of detecting a target, approaching it, holding position above it, and safely returning home**.

The project is divided into two main parts:

- **Flight control logic**
- **Machine learning for target detection**

This separation allows the autonomy system to remain modular and easy to extend.

---

# Project Overview

The drone follows a fully autonomous mission pipeline:

1. Connect to the autopilot through **MAVLink**
2. Perform **VTOL takeoff**
3. Start a **fixed-wing search mission**
4. Scan the ground using a **gimbal camera**
5. Detect a target (simulated for now)
6. Transition from **Fixed Wing → VTOL**
7. Navigate to the detected target
8. Hold position above the target
9. Return to launch and land safely

The mission logic is implemented using a **Finite State Machine (FSM)**.

---

# Repository Structure

```
uav-autonomy/
│
├── flight_control/
│   Core UAV autonomy code
│
├── machine_learning/
│   Future neural network for target detection
```

---

# Flight Control

The `flight_control` folder contains the autonomy system responsible for controlling the drone.

Main components include:

- MAVLink communication with the autopilot
- Autonomous mission pipeline
- State machine controlling the mission
- VTOL transitions
- Target navigation
- Gimbal control
- Mission upload

The autonomy logic is implemented in a **state machine architecture** that manages the different phases of the mission.

Example states include:

- `SEARCH_FW` — fixed-wing search pattern
- `TRACK_DETECTED` — target detection
- `TRANSITION_TO_VTOL` — switch to multirotor mode
- `VTOL_HOLD_OVER_TARGET` — hover above target
- `RETURN_HOME` — safe return and landing

---

# Machine Learning (Future Work)

The `machine_learning` folder will contain the **target detection system**.

The goal is to replace the current **simulated detection** with a neural network capable of detecting objects in real-time from the drone camera.

Future components may include:

- Dataset generation
- Neural network training
- Inference pipeline
- Real-time integration with the autonomy system

The ML module will send detection results to the **flight controller FSM**, which will trigger the target interception sequence.

---

# Simulation

The project can be tested using **ArduPilot SITL (Software-In-The-Loop)**.

This allows testing the entire autonomy pipeline without flying a real drone.

---

# Technologies Used

- Python
- MAVLink
- ArduPilot
- SITL simulation
- Computer vision (future)
- Machine learning (future)

---

# Project Goal

The objective of this project is to explore **autonomous UAV systems**, combining:

- robotics
- flight control
- computer vision
- artificial intelligence

This work focuses on building a **modular autonomy architecture** where perception and control are clearly separated.

---

# Author

Maxime Jolliot  
ISAE-ENSMA Engineering Student  
Autonomous systems and UAV development
