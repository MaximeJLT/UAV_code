# ArduPilot SITL + Python UAV Controller (Complete Setup Guide)

This guide explains how to install and run:

- **ArduPilot SITL simulator**
- **A Python UAV autonomy controller (`controller_fsm.py`)**

The tutorial starts from **a completely clean Windows machine**.

No Python, no Git, no VSCode required.

---

# Overview

The workflow is:

1. Install **WSL (Linux inside Windows)**
2. Install **ArduPilot SITL**
3. Run the **QuadPlane simulator**
4. Run the **Python controller (`controller_fsm.py`)**
5. Test autonomous missions

---

# 1. Install WSL

Open **PowerShell as Administrator**.

Run:

```powershell
wsl --install
```

Restart your computer.

---

# 2. First Ubuntu Launch

Open **Ubuntu** from the Windows Start Menu.

Create a Linux user.

Example:

```
username: maxime
password: ********
```

---

# 3. Update Ubuntu

Inside the Ubuntu terminal run:

```bash
sudo apt update
sudo apt upgrade -y
```

---

# 4. Install Required Dependencies

Run:

```bash
sudo apt install git python3 python3-pip python3-dev python3-opencv python3-wxgtk4.0 python3-matplotlib python3-lxml python3-yaml python3-pyserial python3-argparse python3-empy python3-future python3-setuptools python3-numpy python3-pygame build-essential ccache gawk -y
```

---

# 5. Download ArduPilot

Go to the home directory:

```bash
cd ~
```

Clone the ArduPilot repository:

```bash
git clone https://github.com/ArduPilot/ardupilot.git
```

Enter the folder:

```bash
cd ardupilot
```

---

# 6. Install ArduPilot Tools

Run:

```bash
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

Reload the environment:

```bash
. ~/.profile
```

---

# 7. Build ArduPlane SITL

Go to the ArduPlane directory:

```bash
cd ~/ardupilot/ArduPlane
```

Configure SITL:

```bash
../waf configure --board sitl
```

Build ArduPlane:

```bash
../waf plane
```

---

# 8. Run the QuadPlane Simulator

Go back to the root folder:

```bash
cd ~/ardupilot
```

Run SITL:

```bash
Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane --console --map
```

This launches:

- the **ArduPilot QuadPlane simulator**
- the **MAVProxy console**
- the **map window**

Leave this terminal running.

---

# 9. Prepare the UAV Python Project

Open a **second Ubuntu terminal**.

Create a project folder:

```bash
cd ~
mkdir -p ~/projects
cd ~/projects
```

Copy your UAV project into WSL.

Example if the project is on your Windows Desktop:

```bash
cp -r /mnt/c/Users/YOUR_WINDOWS_USER/Desktop/uav_ground_station ~/projects/
```

Enter the project folder:

```bash
cd ~/projects/uav_ground_station
```

---

# 10. Create a Python Virtual Environment

Create a virtual environment:

```bash
cd ~/projects
python3 -m venv venv-ardupilot
```

Activate it:

```bash
source ~/projects/venv-ardupilot/bin/activate
```

---

# 11. Install Python Dependencies

Inside the project folder:

```bash
cd ~/projects/uav_ground_station
```

Install required packages:

```bash
pip install pymavlink
```

or if your project includes a requirements file:

```bash
pip install -r requirements.txt
```

---

# 12. Run the UAV Autonomy Controller

Once SITL is running, launch the Python controller.

In the **second terminal**:

```bash
cd ~/projects/uav_ground_station
source ~/projects/venv-ardupilot/bin/activate
python controller_fsm.py
```

This script will:

1. Connect to the UAV through MAVLink
2. Arm the drone
3. Perform a **VTOL takeoff**
4. Start a **fixed-wing search mission**
5. Detect a target (simulated)
6. Transition back to **VTOL**
7. Navigate to the target
8. Hold position above it
9. Return home and land

---

# 13. Running the Full System

You always need **two terminals**.

Terminal 1 → start SITL

```bash
cd ~/ardupilot
Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane --console --map
```

Terminal 2 → run the UAV controller

```bash
cd ~/projects/uav_ground_station
source ~/projects/venv-ardupilot/bin/activate
python controller_fsm.py
```

---

# 14. Creating Custom Missions (Mission Planner)

To test different trajectories, missions must be created in **Mission Planner**.

Mission Planner can connect to SITL through **UDP port 14550**.

### Steps

1. Open **Mission Planner**
2. Connect to:

```
UDP
127.0.0.1
14550
```

3. Go to the **Flight Plan** tab.

---

# Important: Waypoint 0

Mission Planner always creates **Waypoint 0**.

Waypoint 0 **must be located at the SITL home position**.

In SITL, the home location is defined when the simulator starts.

Example:

```
47.397742 8.545594
```

So:

- Waypoint 0 should stay at the **home location**
- Do not move waypoint 0 somewhere else

The drone will automatically start the mission from **Waypoint 1**.

---

# Creating an Infinite Mission Loop

To create a mission that repeats forever, add a final command:

```
DO_JUMP
```

In Mission Planner:

```
Command: DO_JUMP
Target WP: 1
Repeat: -1
```

This command jumps back to waypoint 1 indefinitely.

This allows:

- infinite search patterns
- continuous testing in SITL

---

# 15. Simulated Target Detection

Inside the controller code there is a **simulated target detection**.

After a delay, the script prints:

```
SIMULATED TARGET DETECTED
```

This is used for testing the **VTOL target approach logic**.

The detection is implemented as a timer in the controller.

---

# Disabling the Simulated Detection

If you want to disable it, open:

```
controller_fsm.py
```

Find the block:

```python
# Simulation detection
if time.time() - start_time > 50.0:
    print("SIMULATED TARGET DETECTED")
    target_latlon = (last_lat, last_lon)
    state = State.TRACK_DETECTED
```

Comment it:

```python
# if time.time() - start_time > 50.0:
#     print("SIMULATED TARGET DETECTED")
#     target_latlon = (last_lat, last_lon)
#     state = State.TRACK_DETECTED
```

This allows you to integrate a **real detection system** (camera + neural network).

---

# 16. Useful Linux Commands

Go to home:

```bash
cd ~
```

Show current folder:

```bash
pwd
```

List files:

```bash
ls
```

Enter a folder:

```bash
cd folder_name
```

Go up one directory:

```bash
cd ..
```

---

# Final Result

You now have a complete **UAV autonomy testing environment**:

- ArduPilot SITL
- QuadPlane simulation
- Python MAVLink controller
- Autonomous mission execution
- Target approach logic

This allows testing the full autonomy pipeline safely before flying the real UAV.
