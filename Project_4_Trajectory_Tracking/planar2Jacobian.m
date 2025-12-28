function J = planar2Jacobian(x, params)
    % planar2Jacobian: Computes the Jacobian matrix J(q) for a 2-DOF planar manipulator.
    %
    % Inputs:
    %   x      : State vector containing joint angles [q1; q2; dq1; dq2] or
    %            at minimum the first two elements [q1; q2].
    %   params : Robot parameters
    % Output:
    %   J      : Jacobian matrix 
    % Extract joint angles

    q1 = x(1);
    q2 = x(2);

    l1 = params.l1;
    l2 = params.l2;

    % Geometric Jacobian mapping joint velocities to end effector linear velocity
    J11 = -l1*sin(q1) - l2*sin(q1 + q2);
    J12 = -l2*sin(q1 + q2);
    J21 =  l1*cos(q1) + l2*cos(q1 + q2);
    J22 =  l2*cos(q1 + q2);

    J = [J11 J12;
         J21 J22];
end