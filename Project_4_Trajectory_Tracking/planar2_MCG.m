function [M, Cdq, G] = planar2_MCG(q, dq, params)

    % planar2_MCG: Computes the dynamic components of a 2-DOF planar robot.
    %
    % Inputs:
    %   q      : Joint angles        [q1; q2]   
    %   dq     : Joint velocities    [dq1; dq2]

    q1 = q(1);  q2 = q(2);
    dq1 = dq(1); dq2 = dq(2);
   
    %   params : Robot parameters structure containing:
     l1  = params.l1;
    lc1 = params.lc1;
    lc2 = params.lc2;
    m1  = params.m1;
    m2  = params.m2;
    I1  = params.I1;
    I2  = params.I2;
    g   = params.g;
    % Outputs:
    %   M   :  inertia  matrix
     d11 = m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(q2)) + I1 + I2;
    d12 = m2*(lc2^2 + l1*lc2*cos(q2)) + I2;
    d22 = m2*lc2^2 + I2;

    M = [d11 d12;
         d12 d22];
    %   Cdq : Coriolis/centrifugal vector  
     h = -m2*l1*lc2*sin(q2);

    Cdq1 = h*(2*dq1*dq2 + dq2^2);  
    Cdq2 = h*(-dq1^2);

    Cdq = [Cdq1;
           Cdq2];

    %   G   : gravity vector
    g1 = (m1*lc1 + m2*l1)*g*cos(q1) + m2*lc2*g*cos(q1 + q2);
    g2 = m2*lc2*g*cos(q1 + q2);

    G = [g1;
         g2];

end