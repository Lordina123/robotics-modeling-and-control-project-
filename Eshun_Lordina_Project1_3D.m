%% Project 1.2: Commutative Property in 3D
% Course: Robotics: Modeling, Planning & Control
% The University of Memphis
% Contact: hhafezi@memphis.edu; mdavoodi@memphis.edu
% Student Name:Lordina Eshun 
%
% In this project, you will explore whether 3D rigid-body rotations 
% are commutative. The coordiante frame is attached to cube for better
% underestanding the rotaion process.
%
% Task:
%   - Complete the missing parts of the code (Look for Q1-Q11)
%   - Visualize a cube under different rotation orders
%   - Compare the resulting frames and explain

clear; close all; clc;

%% --- Define cube (box) in local frame ---
dx = .4; dy = .2; dz = .1;
V = [ -dx/2 -dy/2 -dz/2;
       dx/2 -dy/2 -dz/2;
       dx/2  dy/2 -dz/2;
      -dx/2  dy/2 -dz/2;
      -dx/2 -dy/2  dz/2;
       dx/2 -dy/2  dz/2;
       dx/2  dy/2  dz/2;
      -dx/2  dy/2  dz/2 ];

% Convert cube's points to homogeneous coordinates:
Vh = [V ones(8,1)]';
%% Rotation Matrix and transformation matrix
%Q1. Complete the following function definitions starting at line 138:
% 1. rotx3D(theta)
% 2. roty3D(theta)
% 3. rotz3D(theta)
% 4. tform3D(R,x,y,z)


%% --- Initial Frame (subplot 1) ---
world_frame(1)   
Rx = rotx3D(0);
Ry = roty3D(0);
Rz = rotz3D(0);
% No rotation (one of rotations can be used)
R_total = Rx
T = tform3D(R_total,0,0,0);
drawFrame3D(T,'0')
hold on
drawBox3D(Vh')
%% --Rotation about the Current X axis (subplot 2)--
world_frame(2)
% Q2. Compute the rotation matrix for a pi/2 rotation about the current x-axis
Rx = rotx3D(pi/2);
Ry = roty3D(0);
Rz = rotz3D(0);
% Q3. Compute the homogeneous transformation matrix representing a π/2 rotation about the current x-axis
R_total =Rx;
T = tform3D(R_total,0,0,0);

drawFrame3D(T,'Rx')  
hold on
% Use T and Vh to rotate the box
Vh_new = T*Vh;
drawBox3D(Vh_new')

%% --Rotation about the Current Y axis (subplot 3)--
world_frame(3)
% Q4. Compute the rotation matrices for the following sequence:
% (a) A pi/2 rotation about the current x-axis
% (b) Followed by a pi/2 rotation about the current y-axis
Rx = rotx3D(pi/2);
Ry = roty3D(pi/2);
Rz = rotz3D(0);

% Q5. Compose the two rotations from Q3 and compute the resulting homogeneous transformation matrix
R_total = Ry * Rx;
T = tform3D(R_total,0,0,0);

drawFrame3D(T,'RxRy')
hold on
%Use T and Vh to rotate the box
Vh_new = T*Vh;
drawBox3D(Vh_new')

%% --Initial Frame (subplot 4)--
world_frame(4)   
Rx = rotx3D(0);
Ry = roty3D(0);
Rz = rotz3D(0);
% No rotation (one of rotations can be used)
R_total = Rx
T = tform3D(R_total,0,0,0);
drawFrame3D(T,'0')
hold on
drawBox3D(Vh')

%% ----Rotation about the Y axis (subplot 5)--
world_frame(5)
% Q6. Compute the rotation matrix for a Pi/2 rotation about the current y-axis
Rx = rotx3D(0);
Ry = roty3D(pi/2);
Rz = rotz3D(0);
% Q7. Compute the homogeneous transformation matrix corresponding to a pi/2 rotation about the current y-axis
R_total =Ry ;
T = tform3D(R_total,0,0,0);

drawFrame3D(T,'Ry')
hold on
%Use T and Vh to rotate the box
Vh_new = T*Vh;
drawBox3D(Vh_new')

%% ----Rotation about the X axis (subplot 6)--
world_frame(6)
% Q8. Compute sequential rotation matrices:
%     - First rotate Pi/2 about the current y-axis (R_y)
%     - Then rotate Pi/2 about the current x-axis (R_x)
Rx = rotx3D(pi/2);
Ry = roty3D(pi/2);
Rz = rotz3D(0);
% Q9. Compose the two rotations from Q7 and compute the resulting homogeneous transformation matrix
R_total = Rx * Ry;
T = tform3D(R_total,0,0,0);

drawFrame3D(T,'RyRx')
hold on
% Use T and Vh to rotate the box
Vh_new = T*Vh;
drawBox3D(Vh_new')

%% --- Student Questions ---
% Q10. Compare the frames in subplot 1-3 and subplot 4-6.
%     Are they the same? Why or why not?
% No they are not the same. Rx*Ry is not the same as Ry*Rx. This is because
% 3D rotations are not commutative. The order of the 3D rotation matters.
%
% Q11. Do rotations about different axes commute in 3D?
%     How is this different from the 2D case?
% Rotation about Different axis do not commute in 3D. 3D rotations involves
% multiple axes that reorient after each rotation. In 2D, all rotations are
% about the same axis do they commute.

%% ---Functions---
function R = rotx3D(theta)
    % Rotation matrix about X-axis
    R = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
end

function R = roty3D(theta)
    % Rotation matrix about Y-axis
    R = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
end

function R = rotz3D(theta)
    % Rotation matrix about Z-axis
    R = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
end

function T = tform3D(R,x,y,z)
    %Homogeneous transformation matrix
    t = [x; y; z];
    T = [R t; 0 0 0 1];
end

% No need to change following fucntions.
function drawFrame3D(T, name)
    % T: 4x4 homogeneous transform
    % name: string label for the frame
    
    % origin
    o = T(1:3,4);
    % axes
    x_axis = T(1:3,1);
    y_axis = T(1:3,2);
    z_axis = T(1:3,3);

    hold on;
    scale = .5;
    quiver3(o(1),o(2),o(3), x_axis(1),x_axis(2),x_axis(3), scale, ...
        'r','LineWidth',2,'MaxHeadSize',0.5); % X (red)
    quiver3(o(1),o(2),o(3), y_axis(1),y_axis(2),y_axis(3), scale, ...
        'g','LineWidth',2,'MaxHeadSize',0.5); % Y (green)
    quiver3(o(1),o(2),o(3), z_axis(1),z_axis(2),z_axis(3), scale, ...
        'b','LineWidth',2,'MaxHeadSize',0.5); % Z (blue)
    
    % --- Add labels ---
    s = .01;
    text(o(1)+x_axis(1)*scale+s, o(2)+x_axis(2)*scale+s, o(3)+x_axis(3)*scale+s, ...
        'X', 'Color','r','FontSize',10);
    text(o(1)+y_axis(1)*scale+s, o(2)+y_axis(2)*scale+s, o(3)+y_axis(3)*scale+s, ...
        'Y', 'Color','g','FontSize',10);
    text(o(1)+z_axis(1)*scale+s, o(2)+z_axis(2)*scale+s, o(3)+z_axis(3)*scale+s, ...
        'Z', 'Color','b','FontSize',10);
    
    % add label
    text(o(1)-.2,o(2)+.2,o(3)+.2, sprintf('Frame %s',name), 'FontSize',10,'FontWeight','bold');
end

function world_frame(i)
subplot(2,3,i); hold on; grid on; 
xlim([-.5,.5])
ylim([-.5,.5])
zlim([-.5,.5])
xlabel('X'); ylabel('Y'); zlabel('Z');
view(135,30);

end

function drawBox3D( V)

    % Faces of the cube
    F = [1 2 3 4;  % bottom
         5 6 7 8;  % top
         1 2 6 5;  % side
         2 3 7 6;
         3 4 8 7;
         4 1 5 8];
    
    % Draw patch
    patch('Vertices',V(:,1:3), 'Faces',F, ...
          'FaceColor','b', 'FaceAlpha',.7, ...
          'EdgeColor','k');
end
