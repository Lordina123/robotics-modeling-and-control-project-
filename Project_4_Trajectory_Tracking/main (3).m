clear; close all;
%% Project 4: 2-DOF Planar Manipulator Control
%contact: hhafezi@memphis.edu

% In this project, you will design and simulate control laws for a 2-DOF
% planar manipulator such that its end-effector follows a desired Cartesian
% trajectory (specifically, a figure-8 or infinity-shaped path). The project
% consists of the following three main tasks:

% 1. Generate a trajectory in Cartesian space, including the desired
%    end-effector position, velocity, and acceleration. Then compute the
%    corresponding desired joint trajectories.

% 2. Design and implement two types of controllers:
%    a) Joint-space PD controller
%    b) Cartesian-space PD controller
%    Use both controllers to generate the required joint torque inputs (tau)
%    to enable the robot to follow the desired trajectory.

% 3. Use the forward kinematics of the robot to simulate and visualize the
%    robot's motion over time. Compare the performance of the two controllers
%    in terms of tracking accuracy in both joint space and Cartesian space.

%% Robot Parameters
% Use the following physical and inertial properties for the 2-DOF planar manipulator:

params.l1  = 1.0;   % Length of link 1 (m)
params.l2  = 1.0;   % Length of link 2 (m)
params.lc1 = 0.5;   % Distance from joint 1 to center of mass of link 1 (m)
params.lc2 = 0.5;   % Distance from joint 2 to center of mass of link 2 (m)
params.m1  = 1.0;   % Mass of link 1 (kg)
params.m2  = 1.0;   % Mass of link 2 (kg)
params.I1  = 0.1;   % Moment of inertia of link 1 about its center of mass (kg·m^2)
params.I2  = 0.1;   % Moment of inertia of link 2 about its center of mass (kg·m^2)
params.g   = 9.81;  % Acceleration due to gravity (m/s^2)
%% Trajectory Generation

% Trajectory Parameters
A = 0.7;    % Amplitude in x-direction
B = 0.7;    % Amplitude in y-direction
T = 5;      % Duration of the motion (seconds)
dt = 0.01;  % Time step
t = 0:dt:T; % Time vector

x0 = 0.8;   % Center offset in x
y0 = 0.8;   % Center offset in y

% Q.1. Infinity shape Trajectoy: Go to funciton 'generateInfinityTrajectory.m'
% complete it. 
[p_ref,dp_ref,ddp_ref] = generateInfinityTrajectory(x0,y0,A,B,T,dt);



% Q.2. Using the given Cartesian trajectory 'p_ref', compute the corresponding
% joint-space trajectory 'q_ref'. To achieve this, you must implement the following
% two functions:
%
%   1. Forward Kinematics: Complete the function 'planar2FW.m'. This function
%      should compute the positions of all joints.
%
%   2. Inverse Kinematics: Complete the function 'Planar2IK.m'. This function
%      should compute the joint angles required to reach a desired end-effector
%      position. Note that you must use your 'planar2FW' function inside
%      'Planar2IK.m'.
%   3. Then complete the for loop here to find q_ref trajectory.
%
% Q.3. How do you prevent the solution from switching between the elbow-up and
% elbow-down configurations during inverse kinematics?

q_ref = zeros(2, length(t));
q0=[0;0];
for i=1:length(t)

 fun = @(q) Planar2IK(q,p_ref(:,i),params);
 options = optimset('Display', 'off');
    q_ref(:,i) = fminsearch(@(q) Planar2IK(q, p_ref(:,i), params), q0, options);
    
    q0 = q_ref(:,i)
end
%Q3 ans:  By using the previous solution as the initial guess for the next time step (q0 = q_ref(:,i)), 
% we ensure the optimizer finds the closest solution, maintaining continuity and preventing sudden switches between elbow-up and elbow-down configurations.

% Q.4. Find the dq_ref and ddq_ref. To achive this goal you will need the
%jacabian and its derivative using fucntion %planar2Jacovian.m and 
% planar2dJacovian.m. 

dq_ref  = zeros(2, length(t));
ddq_ref = zeros(2, length(t));

for i=1:length(t)

    x_tmp = [q_ref(:,i); 0; 0];
    J = planar2Jacobian(x_tmp, params);

    dq_ref(:,i) = J \ dp_ref(:,i);
end

for i = 1:length(t)
    x_tmp = [q_ref(:,i); dq_ref(:,i)];
    J  = planar2Jacobian(x_tmp, params);
    dJ = planar2dJacobian(x_tmp, params);

    ddq_ref(:,i) = J \ (ddp_ref(:,i) - dJ*dq_ref(:,i));

end


%% Forward Dynamic and Control

% Q.5. In this section, you are required to implement the control loop that
% performs the following parts:

% Initial state: [q1; q2; dq1; dq2]
X0 = [q_ref(:,1);0;0];
% Initial torque and state storage
tau = zeros(2, length(t));
Q = zeros(4, length(t));
tau(:,1) = [0;0];
Q(:,1) = X0;
%Defining the controllers gain
Kp=100 %gains increased
Kd= 20 %gains increased

for i=1:length(t)

% Time span for this control update
tspan = [(i-1)*dt, i*dt];

% Q.5.1. Implement the forward dynamics over one time step using ode45.
% To do this, you must complete the robot's dynamic model inside the
% function 'planar2dynamics.m'.
[~, X] = ode45(@(t,x) planar2dynamics(t,x,tau(:,i), params), tspan, X0);

% Q.5.2. Implement the joint-based controller in the file 'joint_PD_controller.m'
% to generate the required torque for the robot to track the desired
% trajectory. % To do so, you also need another function ('planar2_MCG') to compute the 
% different components of the robot’s dynamics, including the inertia 
% matrix M, the Coriolis/centrifugal term C*dq, and the gravity vector G.
 x_curr = X(end,:).'
ref_joint = [q_ref(:,i);
                 dq_ref(:,i);
                 ddq_ref(:,i)];
ref_cart = [p_ref(:,i);
                dp_ref(:,i);
                ddp_ref(:,i)];
%tau(:,i+1) = joint_PD_control(X(end,:),.....);
  tau(:,i+1) = joint_PD_control(x_curr, ref_joint, Kp, Kd, params);
% Note: The output of ode45 will be a matrix of size (N x 4), where each row
% corresponds to the state at a time step. You must use the **last row**
% (i.e., the final state) as the updated state for the next iteration.

% Q.5.3. Implement the Cartesian-based controller in the file 'cartesian_PD_control.m'
% to generate the required torque for the robot to track the desired
% trajectory. Same as joint controller, you need planar2_MCG funciton.
%tau(:,i+1) = cartesian_PD_control(X(end,:),......);

% Choose which controller to use by commenting or uncommenting the appropriate line.

%updating initial value and store the state
X0 = X(end,:);
Q(:,i) = X0'; 
end

%% Plot q_ref, q, dq_ref, dq_ref

% Q.6. In this part, plot the joint angles along with their reference
% trajectories. Are they close? If not, adjust the PD controller gains to
% improve the tracking performance.
subplot(2,1,1);
plot(t, q_ref(1,:), 'r--', 'LineWidth', 2); hold on;
plot(t, Q(1,:), 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); 
ylabel('q_1 (rad)');
legend('Reference', 'Actual');
title('Joint 1 Position');
grid on;

% Joint 2 Position
subplot(2,1,2);
plot(t, q_ref(2,:), 'r--', 'LineWidth', 2); hold on;
plot(t, Q(2,:), 'b-', 'LineWidth', 1.5);
xlabel('Time (s)'); 
ylabel('q_2 (rad)');
legend('Reference', 'Actual');
title('Joint 2 Position');
grid on;

%% Animate the 2R planar arm motion

% Q.7. Given the robot's joint angles q, plot the robot's motion using the
% forward kinematics. Also plot the end-effector trajectory in 2D, including
% both the reference end-effector path and the actual path generated by the
% forward dynamics.
p_actual = zeros(2, length(t));
for i = 1:length(t)
    [~, ~, p3] = Planar2FK(Q(1:2,i), params);
    p_actual(:,i) = p3;
end

% Create animation figure
figure;

% Animation loop - THIS DRAWS THE ROBOT MOVING
for i = 1:10:length(t)  % Every 10th frame for speed
    
    % Clear figure and redraw everything
    clf;
    hold on;
    axis equal; 
    grid on;
    xlim([0, 2]); 
    ylim([0, 2]);
    xlabel('X (m)'); 
    ylabel('Y (m)');
    title(sprintf('Time: %.2f s', t(i)));
    
    % Plot full reference path (stays on screen)
    plot(p_ref(1,:), p_ref(2,:), 'r--', 'LineWidth', 2, 'DisplayName', 'Reference');
    
    % Plot actual path traced so far (grows with time)
    plot(p_actual(1,1:i), p_actual(2,1:i), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual');
    
    % Get current robot configuration
    [p1, p2, p3] = Planar2FK(Q(1:2,i), params);
    
    % Draw the robot arm at current position
    plot([p1(1), p2(1), p3(1)], [p1(2), p2(2), p3(2)], ...
         'ko-', 'LineWidth', 4, 'MarkerSize', 10, 'MarkerFaceColor', 'b');
    
    % Highlight end-effector
    plot(p3(1), p3(2), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
    
    legend('Location', 'best');
    
    % Update the figure (this makes it animate!)
    drawnow;
    
    % Small pause for smooth animation
    pause(0.02);
end


