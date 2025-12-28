% Projcet 1.1: Commutative Property in 2D
%Course: Robotics: Modeling, Planning & Control
%The University of Memphis
%Contact: hhafezi@memphis.edu; mdavoodi@memphis.edu
% Student Name: Lordina Eshun                

% In this project, you will explore whether 2D rigid-body motions
% (rotations + translations) are commutative.
%
% Task:
%   - Complete the missing parts of the code
%   - Visualize frames A and B under different composition orders

clear; clc; close all;
figure; hold on; axis equal; grid on;
xlim([-2 2]); ylim([-2 2]);
xlabel('X'); ylabel('Y'); title('2D Frames: World, A, B');
% World frame
quiver(0,0,1,0,1,'k','LineWidth',2);  % x
quiver(0,0,0,1,1,'k','LineWidth',2);  % y
text(1,0,'X0','FontSize',10); text(0,1,'Y0','FontSize',10);



%% --- Define Transformations ---
%Q1. Fill the function tform2D started at line 53 to generate a homogeneous transformation T given theta(rad) and translation (tx,ty).

%Q2. Find a transformation for pure translation tx=0.5 and ty=0.5  
T_t = tform2D(0, 0.5, 0.5);

%Q3. Find transformation for a pure rotation of 2pi/3 about the current frame. 
T_R = tform2D(2*pi/3,0,0);

% Use the previously calculated T_t and T_R in the next two questions to:
%Q4. Find the transformation of the translation [0.5, 0.5]' along the current frame, followed by the rotation 2pi/3 about the current frame 
T_tR = T_R *T_t;
drawFrame2D(T_tR,'b','T_tR');

%Q5. Find the transformation of the rotation 2pi/3 about the current frame, followed by the traslation [0.5, 0.5]' along current frame 
T_Rt = T_t*T_R;
drawFrame2D(T_Rt,'b','T_Rt');

% Q6: Explain the resulting new coordinate frames (how they are different)
% Ans: The frames T_tR and T_Rt are different.  The frames T_tR and T_Rt are different, indicating that 2D rigid-body motions
% (rotation and translation) are not commutative. The order in which you apply them changes the final position and orientation of the frame.



%% ---Funcitons---

function T = tform2D(theta, tx, ty)
    % 2D homogeneous transform from rotation (rad) and translation 
    % Given the rotation theta and translation tx and ty complete the
    % rotation matrix T and Homoheneous transformation T
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    T = [R [tx;ty]; 0 0 1]
end


% No need to change the following function
function drawFrame2D(T, color, name)
    % T: 3x3 homogeneous transform (rotation + translation)
    % color: base color (e.g. 'r' or 'b')
    % name: optional label for the frame
    
    % origin (translation)
    o = T(1:2,3);  
    
    % new x- and y-axes basis vectors (from rotation columns)
    x_axis = T(1:2,1);  
    y_axis = T(1:2,2);
    
    % draw X axis
    quiver(o(1), o(2), x_axis(1), x_axis(2), 1, ...
           'Color','r','LineWidth',2,'MaxHeadSize',0.5);
    % draw Y axis
    quiver(o(1), o(2), y_axis(1), y_axis(2), 1, ...
           'Color','g','LineWidth',2,'MaxHeadSize',0.5);
    % add name for each coordinate
    text(T(1,3)+T(1,1)+.01,T(2,3)+T(2,1)+.01,strcat('X', name),'Color','r','FontSize',10);
    text(T(1,3)+T(1,2)+.01,T(2,3)+T(2,2)+.01,strcat('Y', name),'Color','r','FontSize',10);
       
    % add text label if provided
    if nargin > 2
        text(o(1)+.02, o(2)-.02, name, 'FontSize',10,'Color',color);
    end
end
