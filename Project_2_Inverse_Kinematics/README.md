## Project 2: 6-DOF Manipulator Inverse Kinematics (DH + fsolve)

This project models a 6-DOF robotic manipulator using the standard Denavit–Hartenberg (DH) convention, computes the forward kinematics to get the end-effector pose, and then solves the inverse kinematics problem using `fsolve` to find a joint configuration that reaches a desired target pose.

The robot has 6 joints where Joint 3 is prismatic (represented by d3), and the other joints are revolute (represented by θ1, θ2, θ4, θ5, θ6). The end-effector pose is represented as position (x, y, z) plus orientation (φ, θ, ψ) extracted from the rotation matrix.

### Files
- **DH.m**
  Builds the homogeneous transformation matrix between consecutive links using the *standard* DH parameters (a, α, d, θ).

- **forward_kinematic.m**
  Computes the full transform from base to end-effector (H06), then returns the end-effector pose:
  `[x; y; z; phi; theta; psi]`.

- **main_inverse_kinematics.m**
  Defines the DH parameters, sets a desired end-effector pose, uses `fsolve` to solve IK by minimizing pose error, and visualizes the initial and solved configurations.

- **robot_plot.m**
  Visualizes the robot in 3D by computing link-frame transforms and drawing each link as line segments. Can also show the desired end-effector position.

### Tools Used
- MATLAB (Optimization Toolbox for `fsolve`)
- Standard DH modeling
- Forward and inverse kinematics
- 3D visualization

