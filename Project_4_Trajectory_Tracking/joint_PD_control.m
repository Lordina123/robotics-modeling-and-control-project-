function tau = joint_PD_control(x,ref, Kp, Kd, params)

    % joint_PD_control: Joint-space PD controller with full dynamic compensation
    %
    % Inputs:
    %   x     : Current robot state vector [q1; q2; dq1; dq2]
    %   ref   : Desired joint-space reference [q_ref; dq_ref; ddq_ref]
    %   Kp    : Proportional gain matrix
    %   Kd    : Derivative gain matrix
    %   params: Robot parameters 
    %
    % Output:
    %   tau   : Control torque vector [tau1; tau2]
   % τ=M(q)(q¨​d​+Kd​(q˙​d​−q˙​)+Kp​(qd​−q))+C(q,q˙​)q˙​+G(q)

   q  = x(1:2);    % 2x1
    dq = x(3:4);    % 2x1

    % desired state
    q_ref   = ref(1:2);   % 2x1
    dq_ref  = ref(3:4);   % 2x1
    ddq_ref = ref(5:6);   % 2x1

    % make gains proper matrices if they are scalars
    if isscalar(Kp)
        Kp = diag([Kp Kp]);   % 2x2
    end
    if isscalar(Kd)
        Kd = diag([Kd Kd]);   % 2x2
    end

    % errors (all 2x1)
    e  = q_ref  - q;
    de = dq_ref - dq;

    % robot dynamics
    [M, Cdq, G] = planar2_MCG(q, dq, params);  % M 2x2, Cdq 2x1, G 2x1

    % desired joint acceleration (2x1)
    ddq_des = ddq_ref + Kd*de + Kp*e;

    % computed torque (2x1)
    tau = M*ddq_des + Cdq + G;
end 


