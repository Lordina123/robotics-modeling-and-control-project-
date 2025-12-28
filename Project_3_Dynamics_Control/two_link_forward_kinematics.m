function [p1, p2, p3] = two_link_forward_kinematics(q, params)
q1 = q(1);
q2 = q(2);
l1 = params.l1;
l2 = params.l2;
% Compute positions of 2R planar manipulator (p1 is base, p2 end of link1 and
% p3 is end of link2
p1 = [0; 0];
p2 = [ l1 * cos(q1);
       l1 * sin(q1) ];
p3 = [ l1 * cos(q1) + l2 * cos(q1 + q2);
       l1 * sin(q1) + l2 * sin(q1 + q2) ] ;
end