# 🤖 3D Robot Kinematics Simulator

A professional, open-source 3D Forward Kinematics Simulator for Serial Manipulator Robot Arms developed in **Python**, utilizing **Tkinter** for the dynamic control dashboard and **Matplotlib (mplot3d)** for solid CAD-like 3D rendering.

Designed for engineering students, robotics researchers, and educators to visualize Forward Kinematics, joint axis orientations, and end-effector spatial coordinates in real time.

---

## ✨ Key Features

- **Dynamic DOF Configuration:** Flexibly scale active Degrees of Freedom from 1 to 6 DOF on the fly.
- **Configurable Joint Motion Axes:** Supports Revolute Horizontal (Yaw), Revolute Vertical (Pitch), and Prismatic (Linear Extension) modes.
- **Adjustable Heavy Pedestal Base:** Ground-fixed industrial base with real-time elevation height control relative to the origin plane.
- **Solid CAD Rendering Engine:** Uses volumetric cylinder links and spherical joint knuckles with strict 3D depth-sorting to prevent occlusion bugs.
- **Interactive 3D Visual Indicators:** Real-time spatial text tags highlighting current joint parameters ($\theta_1, \theta_2 \dots$ or $d_1, d_2 \dots$).
- **Live Spatial Coordinate HUD:** Live output calculation of End-Effector Cartesian coordinates $(X, Y, Z)$ in millimeters.
- **Cross-Platform & Lightweight:** Runs smoothly without heavy CAD framework dependencies or OpenGL memory faults.

---

## 🛠️ Installation & Execution

### Prerequisites
Ensure Python 3.8 or higher is installed on your operating system.

### 1. Clone the Repository
```bash
git clone [https://github.com/YOUR_USERNAME/3D-Robot-Kinematics-Simulator.git](https://github.com/YOUR_USERNAME/3D-Robot-Kinematics-Simulator.git)
cd 3D-Robot-Kinematics-Simulator


2. Install Required Dependencies
Bash


pip install -r requirements.txt
3. Launch the Simulator
Bash


python main.py
🕹️ Dashboard & Controls Guide
Global Configuration:

Active DOF: Adjust the spinbox to add or remove joints (1 to 6).

Base Height: Slide or enter values to elevate the robot pedestal above the ground plane.

Joint Parameter Configuration:

Motion Axis: Select between Revolute - Horizontal (Yaw), Revolute - Vertical (Pitch), or Prismatic (P).

Link Length (L): Set specific link lengths in millimeters.

Variables (θ / d): Use sliders or enter exact numbers to actuate individual joints.

3D Viewport Interaction:

Click and drag with the left mouse button anywhere inside the 3D viewport to orbit around the workspace in full 360°.



* **Repository Name:** `3D-Robot-Kinematics-Simulator`
* **Description:** *An interactive 3D Forward Kinematics simulator and GUI dashboard for serial manipulator robot arms built with Python and Matplotlib.*
* **Topics / Tags:** `robotics`, `kinematics`, `forward-kinematics`, `python`, `3d-simulation`, `robot-arm`, `matplotlib`, `tkinter`, `serial-manipulator`





