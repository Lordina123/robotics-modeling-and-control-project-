function [p1, p2, p3] = Planar2FK(q, params)
% Compute positions of planar manipulator
% Inputs:
%   q = [q1; q2]    joint angles (rad)
%   params    : Robot's parameters
% Outputs:
%   p1, p2, p3  :    2D coordinates of base, elbow, and end-effector

    q1 = q(1);
    q2 = q(2);
    l1 = params.l1;
    l2 = params.l2;
    %positions are given below 
    % Base
    p1 = [0; 0];
    
    % Elbow 
    p2 = [l1 * cos(q1); 
          l1 * sin(q1)];
    
    % End-effector 
    p3 = [l1 * cos(q1) + l2 * cos(q1 + q2); 
          l1 * sin(q1) + l2 * sin(q1 + q2)];
end