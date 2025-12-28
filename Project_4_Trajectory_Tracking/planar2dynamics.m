function dx = planar2dynamics(t, x,tau, params)
    % planar2dynamics: Computes the state derivative for a 2-DOF planar robot.
    % Inputs:
    %   t      : Current time 
    %   x      : Current state vector [q1; q2; dq1; dq2]
    %   tau    : Control torque vector [tau1; tau2]
    %   params : Robot parameter
    % Output:
    %   dx     : State derivative [dq1; dq2; ddq1; ddq2]

      q  = x(1:2);
    dq = x(3:4);

    [M, Cdq, G] = planar2_MCG(q, dq, params);

    ddq = M \ (tau - Cdq - G);

    dx = [dq;
          ddq]; 

end