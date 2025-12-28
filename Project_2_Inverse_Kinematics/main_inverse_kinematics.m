clear all; close all; clc;

%% Project 2: 6-DOF Manipulator Arm
% In this project, you will simulate the inverse kinematics (IK) of a 6-DOF
% robot manipulator by completing the partial codes ().

% 1) Implement DH.m:
%    Write a function H = DH(a, alpha, d, theta) that returns the homogeneous
%    transformation between two consecutive links using the *standard* DH convention.
%    Inputs: a (link length), alpha (link twist), d (link offset), theta (joint angle).

% 2) Implement forward_kinematic.m:
%    Write a function ee = forward_kinematic(Q) that computes the end-effector pose
%    given the joint configuration Q.
%    Joint 3 is *prismatic* (represented by d3 in the DH parameters).
%    Joints 1, 2, 4, 5, and 6 are *revolute* (represented by thetai in the DH parameters).

% 3) Implement plot_robot.m:
%    Write a function plot_robot(Q, ee_des) that draws a simple line-based visualization
%    of the robot for the joint configuration Q. The optional ee_des marks the desired
%    end-effector position as a point.

% 4) Complete this main script:
%    Given a desired end-effector pose, use inverse kinematics to find a valid joint
%    configuration Q that reaches the target. Then visualize the result with plot_robot.

% Note taht: Try to complete the tasks in the same order as listed (1 to 4).
% Start with DH.m, then implement forward_kinematic.m, then plot_robot.m,
% and finally complete the main script. Each step builds on the previous one.
%% Specify DH parameters
global d1 a1 alpha1 
global d2 a2 alpha2
global a3 alpha3 theta3
global d4 a4 alpha4 
global d5 a5 alpha5
global d6 a6 alpha6

% Set parameters
%D-H for links. theta 1, theta 2, d3, theta4, theta 5, and theta 6 are not set, because we need to find them. 
d1=1.3; a1=0;  alpha1=-pi/2; % theta1
d2=1.4; a2=0; alpha2=pi/2; % theta2
        a3=0; alpha3=0; theta3=-pi/2;
d4=0.9; a4=0; alpha4=-pi/2; %theta4
d5=0;   a5=0; alpha5=pi/2; %theta5
d6=0.4; a6=0; alpha6=0; %theta6

%% Desired Pose/ Initial Guess
% Pose where we want the end-effector to be:
x_des = 1.4; y_des = 1.4; z_des = 1.8;
phi_des=0; theta_des=0; psi_des=0;
ee_des = [x_des; y_des; z_des; phi_des; theta_des; psi_des]

%Initial guess for 
theta1=pi/2; theta2=0; d3=0.5; theta4=0; theta5=0; theta6=0;
Q0 = [theta1, theta2, d3, theta4, theta5, theta6]';

% Q. Plot the initial configuration of robot. To do so you need first to
% compelete teh DH and robot_plot funciton:
figure(1)
robot_plot(Q0)

%% Inverse Kinematic using fslove:
% Q: First, complete the forward_kinematic function.
%    Then, to use fsolve for finding a joint configuration that achieves
%    a desired end-effector pose, you will need to define the following function:
error_fun = @(Q) (forward_kinematic(Q) - ee_des);

[Q_opt,FVAL,EXITFLAG] = fsolve(error_fun,Q0);
figure(2)
robot_plot(Q_opt,ee_des(1:3))

%% PhD student only reqired to solve this part of project (optional for other students)
%Q3: Instead of one single end-effector point, we want you create a
% circle trajctory centerd at x = 1.4, y = 1.4 with radius 0.2 (a circle in xy plane with z = z_des = 1.8). The reset of deisred 
% end-effector pose property need to be same as previous qeustion.
%you need to plot the robot's motion over time (animate). at the end plot the
%trajecotry that end-effector pass through

t = linspace(0,2*pi,100);   % parameter around the circle

Q_prev = Q_opt;              % warm-start IK from previous solution
traj = zeros(3,100);

figure(3); clf;
for i=1:100
    x_i = 1.4 + 0.2*cos(t(i));
    y_i = 1.4 + 0.2*sin(t(i));
    z_i = z_des;
    ee_des_i = [x_i; y_i; z_i; phi_des; theta_des; psi_des];

    error_fun_i = @(Q) (forward_kinematic(Q) - ee_des_i);
    Q_prev = fsolve(error_fun_i, Q_prev);

    robot_plot(Q_prev, ee_des_i(1:3));
    hold on;
    traj(:,i) = ee_des_i(1:3);
    plot3(traj(1,1:i),traj(2,1:i),traj(3,1:i),'-','LineWidth',1.5);
    drawnow;
end
