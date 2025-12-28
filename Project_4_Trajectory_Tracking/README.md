## Project 4: 2-DOF Planar Manipulator Control (Figure-8 Tracking)

This project designs and simulates control laws for a 2-DOF planar manipulator to track a desired Cartesian trajectory (a figure-8 / infinity path). The trajectory is generated in task space with desired position, velocity, and acceleration, then the robot is controlled using two approaches:

1) Joint-space PD control with full dynamics compensation (computed torque style)  
2) Cartesian-space PD control using the Jacobian to track the end-effector path in task space

The simulation integrates the robot dynamics over time and visualizes the tracking performance.

### Files
- **main (3).m**
  Main script that sets robot parameters, generates the infinity trajectory, computes reference signals, runs the simulation, and compares joint-space vs Cartesian-space control.

- **generateInfinityTrajectory.m**
  Generates the figure-8 reference in Cartesian space:
  desired position `p_ref`, velocity `dp_ref`, and acceleration `ddp_ref`.

- **joint_PD_control.m**
  Joint-space PD controller with dynamics compensation. Uses tracking error in joint space and computes torques using:
  `tau = M(q)*ddq_des + C(q,dq) + G(q)`.

- **cartesian_PD_control.m**
  Cartesian-space PD controller. Computes task-space tracking error, uses the Jacobian (and its time derivative) to obtain desired joint acceleration, then applies dynamics compensation to compute torques.

- **Planar2FK.m**
  Forward kinematics for the 2-link planar arm (computes link positions and end-effector position).

- **Planar2IK.m**
  Inverse kinematics helper used to match a desired end-effector position (used during trajectory tracking/reference generation).

- **planar2dynamics.m**
  Dynamic model used by the ODE solver to propagate the robot state over time given input torques.

- **planar2_MCG.m**
  Computes the dynamic terms: inertia matrix `M(q)`, Coriolis/centrifugal term `C(q,dq)` (returned as `Cdq`), and gravity vector `G(q)`.

- **planar2Jacobian.m**
  Computes the planar Jacobian `J(q)` that maps joint velocities to end-effector velocity.

- **planar2dJacobian.m**
  Computes the time derivative of the Jacobian `dJ/dt`, used for Cartesian-space acceleration control.

### Tools Used
- MATLAB
- Robot dynamics (M, C, G formulation)
- Jacobian-based Cartesian control
- PD control and trajectory tracking
