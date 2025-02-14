# SCARA Robot Arm Simulation

## Overview
This project implements a **SCARA (Selective Compliance Articulated Robot Arm) robot arm simulation** in **MATLAB** for motion planning and trajectory generation. It simulates circular path motion and converts **Cartesian coordinates to joint space** using inverse kinematics.

## Features
- **SCARA Robot Motion Simulation**: Generates motion trajectories for a 4-DOF SCARA robot.
- **Circular Path Generation**: Computes a circular trajectory for end-effector movement.
- **Inverse Kinematics**: Converts Cartesian coordinates to joint angles.
- **Velocity-Based Time Calculation**: Ensures smooth motion execution.
- **Motion Data Storage**: Saves SCARA joint motion data into `.dat` files.

## Project Structure
```
📂 project_root/
├── 📜 tugas_scara1.m            # Main MATLAB script for SCARA motion planning
├── 📜 circle_gen.m              # Generates circular motion trajectories
├── 📜 TugasMotionDataScara_1.dat # Motion data output file 1
├── 📜 TugasMotionDataScara_2.dat # Motion data output file 2
└── 📝 README.md                 # Project documentation
```

## Dependencies
Ensure you have **MATLAB with Robotics Toolbox** installed.

## Usage
### 1. Generate a Circular Path
```matlab
radius = 2.691;
center_x = -1;
center_y = -1.8;
theta = 0:0.1:2*pi;
x = center_x + radius * cos(theta);
y = center_y + radius * sin(theta);
z = 0*theta + 1.0;
phi = 0*theta;
```
Run `circle_gen.m` to modify the radius and center of the circle.

### 2. Plan SCARA Robot Motion
```matlab
run('tugas_scara1.m');
```
This script:
- Defines SCARA robot parameters.
- Generates trajectory in Cartesian space.
- Converts trajectory into **joint space (θ1, θ2, d3, θ4)**.
- Simulates robot motion.
- Saves motion data into `.dat` files.

### 3. Plot Motion Results
```matlab
figure; plot(t, th1, t, th2, t, d3, t, th4);
legend('θ1', 'θ2', 'd3', 'θ4');
```

## Implementation Details
### Motion Planning
The script follows these steps:
1. **Path Generation**: Defines a circular motion trajectory.
2. **Velocity-Based Time Calculation**: Computes time steps for smooth motion.
3. **Trajectory Planning**: Uses linear interpolation for transition points.
4. **Inverse Kinematics**: Computes SCARA robot joint angles:
   ```matlab
   function [th1, th2, d3, th4] = invkinemscara(d1, a1, a2, d4, x, y, z, phi, elbowconf)
   ```
