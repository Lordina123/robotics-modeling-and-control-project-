function f = Planar2IK(q, p_des, params)
% Inputs:
%   q = [q1; q2]    joint angles (rad)
%   p_des : desired position of ee
%   params    : Robot's parameters
% Outputs:
%  f  :    Objective function for IK


    [~, ~, p_ee] = Planar2FK(q, params);

    e = p_ee - p_des;
    f = e.' * e;
end