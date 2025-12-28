clear; clc; close all;
%% Project 3: 2-DOF Planar Manipulator Dynamics using Euler-Lagrange
%contact: hhafezi@memphis.edu

% In this project, you are required to find the forward dynamics of a 2-DOF planar manipulator
% using the Euler-Lagrange equation. To this end, you need to complete the following steps:

% 1. Using symbolic equations, write the expressions for the Kinetic and Potential 
% energies, and derive the Lagrange equation. Use diff(f(x), x) to compute the 
% required derivatives. From the resulting Euler-Lagrange equation, derive the 
% D, C, and G terms. Make sure these terms are correct.

% 2. Given the symbolic expressions for D, C, and G, and the provided physical 
% properties of the robot, you need to formulate the forward dynamics of the system. 
% Set the initial conditions as X0 = [q1; q2; dq1; dq2] = [pi/4; pi/6; 0; 0], 
% the input torque to zero, and the time span as t = [0 5].
%
% Note: Use ode45 to solve the differential equations of the robot dynamics 
% in state-space form.

% 3. The output of the forward dynamics will be the joint trajectories q1(t) and q2(t). 
% In this part, you need to use these joint values to simulate the robot's motion 
% using the forward kinematics.


%% Part 1: Euler-Lagrage equation: Find D, C and G

% Sybalic paramters that you need to use:
%Note that l1 l2 are the links lenght and lc1 and lc2 are their center of
%masses.
syms q1 q2 dq1 dq2 ddq1 ddq2 real;
syms m1 m2 l1 lc1 l2 lc2 g I1 I2 tau1 tau2 real;

% Q1.1) Jacobians of Link 1 and Link 2
Jvc1 = [ -lc1*sin(q1),                    0;
          lc1*cos(q1),                    0];
Jvc2 = [-l1*sin(q1) - lc2*sin(q1 + q2),  -lc2*sin(q1 + q2);
          l1*cos(q1) + lc2*cos(q1 + q2),   lc2*cos(q1 + q2) ];

dq = [dq1; dq2];
%Q1.2) Translational part of the kinetic energy
K_t = simplify(  1/2 * dq.' * ( m1 * (Jvc1.' * Jvc1) + m2 * (Jvc2.' * Jvc2) ) * dq  );

%Q1.3) Rotational part of the kinetic energy
A1 = [1 0;
      0 0];

A2 = [1 1;
      1 1];

K_r = simplify(0.5 * dq' * (I1 * A1 + I2 * A2) * dq );
%K_r = 1/2 * I1 * dq1^2 + 1/2 * I2 * (dq1 + dq2)^2;

% Total kinetic Energy:
K = K_t +K_r;

%Q1.4) Individual potential energies
P1 = m1 * g * lc1 * sin(q1);
P2 = m2 * g * ( l1 * sin(q1) + lc2 * sin(q1 + q2) );

% Total potential energy
P = P1 +P2

% Lagrangian
L = K - P;

%Q1.5 Euler-Lagrange equations. complete The derivatives
q = [q1; q2];
dq = [dq1; dq2];
ddq = [ddq1; ddq2];
tau = [tau1; tau2];

Eqs = sym(zeros(2,1));
for i = 1:2
    dL_dqi =diff(L, q(i)) ;
    dL_ddqi = diff(L, dq(i));
    ddt_dL_ddqi = diff(dL_ddqi, q1)*dq1 + diff(dL_ddqi, q2)*dq2 ...
                + diff(dL_ddqi, dq1)*ddq1 + diff(dL_ddqi, dq2)*ddq2;

    Eqs(i) = simplify(ddt_dL_ddqi - dL_dqi - tau(i));
end

%Q1.6 Extract D: 
[D, terms] = equationsToMatrix(Eqs, [ddq1 ddq2]);
D

%Q1.7 Extract G
Cq_G = simplify(terms+ tau );
G = simplify(subs(Cq_G, {dq1, dq2}, {0, 0}));
G
%Q1.8 Extract C*dq
C_times_dq = simplify(Cq_G - G);
C_times_dq

%Q1.6 verify that your parameters match what you have seen in class.
G = simplify(subs(Cq_G, {dq1,dq2,tau1,tau2}, {0,0,0,0}));

% Expected inertia matrix
d11_exp = m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(q2)) + I1 + I2;
d12_exp = m2*(lc2^2 + l1*lc2*cos(q2)) + I2;
d22_exp = m2*lc2^2 + I2;
D_exp = [d11_exp, d12_exp; d12_exp, d22_exp];

% Expected gravity vector (note negative sign)
G_exp = [-(m1*lc1 + m2*l1)*g*cos(q1) - m2*lc2*g*cos(q1+q2); ...
         -m2*lc2*g*cos(q1+q2)];

% Expected Coriolis/centrifugal vector
h = -m2*l1*lc2*sin(q2);
C_exp = [-h*(2*dq1*dq2 + dq2^2); h*dq1^2];

verify_D = simplify(D - D_exp)
verify_G = simplify(G - G_exp)
verify_C = simplify(C_times_dq - C_exp)
%% Forward Dynamic


params.D_sym = D;              % symbolic mass matrix
params.C_dq_sym = C_times_dq;  % symbolic Coriolis*velocity term
params.G_sym = G;              % symbolic gravity vector

% Parameters
params.l1 = 1; params.l2 = 1;
params.lc1 = 0.5; params.lc2 = 0.5;
params.m1 = 1; params.m2 = 1;
params.I1 = 0.1; params.I2 = 0.1;
params.g = 9.81;
%Q2.1 Define Torque input as zero
params.tau = [0; 0] ;

%Initial x0
x0 = [pi/4; pi/6; 0; 0];
tspan = [0 5];
%Q2.2 Complete the funciton tow_link_dynamic()

%Q2.3 Solve the robot dynamic:
[t, X] = ode45(@(t,x) two_link_dynamic(t, x, params), tspan, x0);

% Plot q1, q2, dq1, and dq2 over time

figure;
subplot(2,2,1);
plot(t, X(:,1), 'LineWidth', 2);
xlabel('Time (s)'); ylabel('q1 (rad)');
title('Joint 1 Position');
grid on;

subplot(2,2,2);
plot(t, X(:,2), 'LineWidth', 2);
xlabel('Time (s)'); ylabel('q2 (rad)');
title('Joint 2 Position');
grid on;

subplot(2,2,3);
plot(t, X(:,3), 'LineWidth', 2);
xlabel('Time (s)'); ylabel('dq1 (rad/s)');
title('Joint 1 Velocity');
grid on;

subplot(2,2,4);
plot(t, X(:,4), 'LineWidth', 2);
xlabel('Time (s)'); ylabel('dq2 (rad/s)');
title('Joint 2 Velocity');
grid on;

%% Inv the 2R planar arm motion using forward kinematic
figure;
axis equal;
axis([-2 2 -2 2]);
grid on;
hold on;

for i = 1:length(t)
    %Q3.1 compute the two_link_forward_kinematic
    q =  X(i, 1:2).';
    [p1, p2, p3] = two_link_forward_kinematics(q, params);

    %Q3.2 Plot links
    plot([p1(1) p2(1)], [p1(2) p2(2)], 'LineWidth', 3, 'Color', [0.1 0.6 0.9]);
    hold on;
    plot([p2(1) p3(1)], [p2(2) p3(2)], 'LineWidth', 3, 'Color', [0.9 0.3 0.2]);

    %Q3.3 Plot joints
    plot(p1(1), p1(2), 'ko', 'MarkerFaceColor', 'k');
    plot(p2(1), p2(2), 'ko', 'MarkerFaceColor', 'k');
    plot(p3(1), p3(2), 'ko', 'MarkerFaceColor', 'k');

    title(sprintf('Time = %.2f s', t(i)));
    drawnow;
    pause(0.1);

    if i < length(t)
        cla; % clear for next frame
    end
end
 
%Q3.4 What is the possible input tau that keeps the joint at thier initail
%configuration?

% Ans: An input torque τ that balances the gravity vector at the initial
% configuration keeps the joints stationary. That is,  τ = G(q₀)