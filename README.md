# TM5-700 Robotic Arm Simulator

This repository includes the following project reports:
- `Robotics_314551810_Project1.pdf`: Detailed report on Part 1, covering kinematics and basic simulations.
- `Robotics_314551810_Project2.pdf`: Detailed report on Part 2, covering motion planning and advanced features.

## Overview

This repository provides a comprehensive MATLAB-based simulator for the TM5-700 collaborative robotic arm manufactured by Techman Robot Inc. The TM5-700 is a 6-degree-of-freedom (DOF) industrial robot designed for collaborative applications, featuring a reach of approximately 700 mm and payload capacity up to 6 kg. The simulator implements essential robotic functionalities including forward kinematics, inverse kinematics, workspace visualization, and motion planning with trajectory generation.

The simulator is built using MATLAB and leverages the Robotics System Toolbox for kinematic computations where applicable, though core algorithms are implemented from scratch for educational and customization purposes. It supports both interactive GUI-based operation via a MATLAB App and script-based execution for batch processing or integration into larger systems.

## Features

- **Forward Kinematics**: Compute the end-effector pose (position and orientation) given joint angles using Denavit-Hartenberg (DH) parameters.
- **Inverse Kinematics**: Solve for joint angles that achieve a desired end-effector pose, handling multiple solutions and joint limits.
- **Workspace Visualization**: Generate 3D plots of the robot's reachable workspace and display the robot configuration.
- **Motion Planning**: Plan smooth trajectories between waypoints with acceleration, constant velocity, and deceleration phases.
- **Animation and Export**: Animate robot motion in real-time or export animations as GIF files or MP4 videos.
- **Interactive GUI**: User-friendly MATLAB App for parameter input and visualization.
- **Modular Design**: Separate functions for kinematics, visualization, and planning, allowing for easy extension and modification.

## Requirements

- MATLAB R2023a or later
- Robotics System Toolbox (recommended for advanced features, but not strictly required)
- Basic MATLAB toolboxes for plotting and matrix operations

## Installation

1. Clone or download this repository to your local machine.
2. Ensure MATLAB is installed and the required toolboxes are available.
3. Add the repository folder to the MATLAB path or navigate to it in the MATLAB environment.

## Usage

### Interactive GUI Mode

1. Open MATLAB.
2. Navigate to the repository folder.
3. Run the MATLAB App by executing: `robotSimulation`
   Alternatively, double-click the `robotSimulation.mlapp` file in the MATLAB Current Folder window.
4. Use the GUI to input parameters for kinematics, workspace visualization, or motion planning.
5. View results in the integrated plots and animations.

### Script-Based Mode

1. Open MATLAB and navigate to the repository folder.
2. Execute the main script: `main`
3. Follow the command-line prompts to select from the following options:
   - Forward Kinematics: Input joint angles to compute end-effector pose.
   - Inverse Kinematics: Input desired pose to solve for joint angles.
   - Visualize Robot Workspace: Generate and display the 3D workspace.
   - Motion Planning: Plan and visualize trajectories between predefined waypoints.
4. For motion animation, uncomment line 73 in `main.m` to enable real-time animation (note: this may take significant time to compute).

## Technical Details

### Kinematics

The simulator uses the Denavit-Hartenberg (DH) convention for modeling the robot's kinematic chain. The DH parameters for the TM5-700 are defined in `dh_parameters.m`, which returns a table of link lengths, twist angles, offsets, and joint types. Modified DH parameters are computed in `mdh.m` for forward kinematics calculations.

**Forward Kinematics** (`forward.m`): Given joint angles θ = [θ₁, θ₂, θ₃, θ₄, θ₅, θ₆], the function computes the homogeneous transformation matrix T representing the end-effector pose relative to the base frame. This involves successive multiplication of transformation matrices derived from DH parameters.

**Inverse Kinematics** (`inverse.m`): Solves for joint angles given a desired end-effector pose T. The implementation uses analytical methods for 6-DOF manipulators, potentially yielding multiple solutions. `areAnglesRight.m` validates solutions against joint limits and mechanical constraints. `modulate_angles.m` generates equivalent solutions by adding ±360° offsets within limits.

### Workspace Visualization

`drawWorkspace.m`: Samples joint space within limits and computes reachable end-effector positions using forward kinematics. The workspace is plotted as a 3D point cloud, with `drawRobot.m` rendering the robot's geometric model.

### Motion Planning

Motion planning is handled in `motion_planning.m`, which demonstrates trajectory generation between three waypoints (A, B, C) defined in `points_and_times.m`. The process involves:

1. Computing inverse kinematics for each waypoint to obtain candidate joint configurations.
2. Selecting optimal configurations using `select_combination.m`, which minimizes a cost function (`cost.m`) based on joint angle differences.
3. Generating smooth trajectories with `compute_theta.m`, implementing trapezoidal velocity profiles with acceleration, constant velocity, and deceleration phases.
4. Visualizing results with `show_motion.m` and animating via `animate_motion.m` or exporting as GIF with `make_gif.m`.

## File Descriptions

### Core Simulation Files
- `robotSimulation.mlapp`: MATLAB App Designer application providing a graphical interface for simulator interaction.
- `main.m`: Entry-point script for command-line operation, orchestrating user selections and function calls.

### Kinematics Functions
- `forward.m`: Implements forward kinematics using DH parameters.
- `inverse.m`: Solves inverse kinematics analytically, returning multiple solutions.
- `dh_parameters.m`: Defines standard DH parameters for the TM5-700.
- `mdh.m`: Computes modified DH transformation matrices.
- `areAnglesRight.m`: Validates joint angles against physical limits and constraints.

### Visualization Functions
- `drawRobot.m`: Renders the 3D robot model in a MATLAB figure.
- `drawWorkspace.m`: Generates and plots the robot's workspace.

### Motion Planning Functions
- `motion_planning.m`: Main function for planning trajectories between waypoints.
- `points_and_times.m`: Provides target poses and timing parameters for motion planning.
- `compute_theta.m`: Generates joint angle trajectories with velocity profiling.
- `cost.m`: Evaluates cost between joint angle configurations.
- `modulate_angles.m`: Expands solution sets by considering joint periodicity.
- `select_combination.m`: Selects optimal joint configurations for smooth motion.

### Animation Functions
- `animate_motion.m`: Performs frame-by-frame animation of robot motion.
- `make_gif.m`: Exports motion animations as GIF files.
- `show_motion.m`: Plots static or animated robot configurations and end-effector paths.

### Additional Files
- `Robot arm_TM5-700_HW3.2_3D.stp`: STEP file for the robot's 3D CAD model (used for reference or import into CAD software).
- `robot_motion.mp4`: Example video of robot motion animation.

## License

This project is provided as-is for educational and research purposes. Please refer to Techman Robot Inc. for licensing information regarding the TM5-700 hardware and software.

## Contributing

Contributions are welcome. Please submit issues or pull requests for bug fixes, feature enhancements, or documentation improvements.