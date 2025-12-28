function dJ = planar2dJacobian(x, params)
    % planar2dJ: Computes the time derivative of the Jacobian matrix (dJ)
    % Inputs:
    %   x      : Current robot state vector [q1; q2; dq1; dq2]
    %   params : Robot's parameters
    
    % Output:
    %   dJ     :time derivative of the Jacobian matrix 

    q1  = x(1);
    q2  = x(2);
    dq1 = x(3);
    dq2 = x(4);

    l1 = params.l1;
    l2 = params.l2;

    dq12 = dq1 + dq2;

    dJ11 = -l1*cos(q1)*dq1 - l2*cos(q1 + q2)*dq12;
    dJ12 = -l2*cos(q1 + q2)*dq12;
    dJ21 = -l1*sin(q1)*dq1 - l2*sin(q1 + q2)*dq12;
    dJ22 = -l2*sin(q1 + q2)*dq12;

    dJ = [dJ11 dJ12;
          dJ21 dJ22];
end