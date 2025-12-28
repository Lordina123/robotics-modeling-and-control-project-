function [tau] = cartesian_PD_control(x,ref,kp,kd,params)

% Inputs:
%   x   : Current robot state vector [q1; q2; dq1; dq2]
%   ref : Desired Cartesian reference vector [p_ref; dp_ref; ddp_ref]
%   kp  : Cartesian proportional gain
%   kd  : Cartesian derivative gain
%   params : Robot physical parameters 
% Outputs:
%   tau : Control torque vector [tau1; tau2] 

    q = x(1:2)';
    dq = x(3:4)';
    p_ref = ref(1:2);
    dp_ref = ref(3:4);
    ddp_ref = ref(5:6);
    [~, ~, p] = Planar2FK(q, params);
    J = planar2Jacobian(x, params);
    dp = J * dq; 
    e_p = p_ref - p;          
    e_dp = dp_ref - dp;      
    ddp_d = ddp_ref + kd * e_dp + kp * e_p;
    dJ = planar2dJacobian(x, params);
    ddq_d = J \ (ddp_d - dJ * dq);
    [M, Cdq, G] = planar2_MCG(q, dq, params);
    tau = M * ddq_d + Cdq + G;

end
