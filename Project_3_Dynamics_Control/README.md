## Project 3: 2-DOF Planar Manipulator Dynamics (Euler–Lagrange)

This project derives and simulates the **forward dynamics** of a 2-DOF planar (2R) manipulator using the **Euler–Lagrange equation**. The inertia matrix **D(q)**, Coriolis/centrifugal term **C(q, dq)**, and gravity vector **G(q)** are derived symbolically, substituted with the robot’s physical parameters, and then used to simulate motion over time with a numerical ODE solver. The results are plotted (joint positions and velocities) and the robot motion is animated using forward kinematics.

### Files
- **main.m**
  Symbolically derives the dynamic model (D, C, G) using Euler–Lagrange, sets initial conditions and parameters, runs the simulation (forward dynamics), plots joint trajectories, and animates the 2-link arm motion.

- **two_link_dynamic.m**
  ODE function for the forward dynamics. At each time step it evaluates D, C, and G numerically and computes:
  **ddq = D(q)^{-1} (tau − C(q,dq) − G(q))**
  then returns the state derivative for integration.

- **two_link_forward_kinematics.m**
  Computes the planar forward kinematics and returns link endpoint positions (base, end of link 1, end of link 2) for plotting and animation.

### Tools Used
- MATLAB (Symbolic Math Toolbox for derivation, ODE solver for simulation)
- Euler–Lagrange dynamics
- Forward dynamics simulation and animation

