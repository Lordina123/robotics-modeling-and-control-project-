function ee = forward_kinematic(Q)

global d1 a1 alpha1 
global d2 a2 alpha2
global a3 alpha3 theta3
global d4 a4 alpha4 
global d5 a5 alpha5
global d6 a6 alpha6

% Get individual thetas and d from input
theta1=Q(1); theta2=Q(2); d3=Q(3); theta4=Q(4); theta5=Q(5); theta6=Q(6);

%Q: Using the DH, find the successive homogeneous
%transformation matrices:
H01 = DH(a1,alpha1,d1,theta1); 
H12 = DH(a2,alpha2,d2,theta2); 
H23 = DH(a3,alpha3,d3,theta3); 
H34 = DH(a4,alpha4,d4,theta4);
H45 = DH(a5,alpha5,d5,theta5);
H56 = DH(a6,alpha6,d6,theta6);

% Q: Compute the composite transform from frame {0} to {2}
H02 = H01*H12;
%Q: Compute the composite transform from frame {0} to {3}
H03 = H02*H23;
%Q: Compute the composite transform from frame {0} to {4}
H04 = H03*H34;
%Q: Compute the composite transform from frame {0} to {5}
H05 = H04*H45;
%Q: Compute the composite transform from frame {0} to {6}
H06 = H05*H56;
%Q: What is locaiton of end-effector with respect to frame {0}?
endOfLink6 = H06(1:3,4);

%Do not change this part:
R = H06(1:3,1:3);
theta=asin(-R(3,1));
phi=asin(R(3,2)/cos(theta));
psi=asin(R(2,1)/cos(theta));

ee = [endOfLink6(1); endOfLink6(2); endOfLink6(3);...
     phi; theta; psi]; 
