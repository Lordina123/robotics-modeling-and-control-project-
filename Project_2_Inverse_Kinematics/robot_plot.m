function robot_plot(Q, ee_des)

theta1=Q(1); theta2=Q(2); d3=Q(3); theta4=Q(4); theta5=Q(5); theta6=Q(6);

global d1 a1 alpha1 
global d2 a2 alpha2
global a3 alpha3 theta3
global d4 a4 alpha4 
global d5 a5 alpha5
global d6 a6 alpha6

%Q: Using the DH, find the successive homogeneous
%transformation matrices:

H01 = DH(a1,alpha1,d1,theta1); 
H12 = DH(a2,alpha2,d2,theta2); 
H23 = DH(a3,alpha3,d3,theta3); 
H34 = DH(a4,alpha4,d4,theta4);
H45 = DH(a5,alpha5,d5,theta5);
H56 = DH(a6,alpha6,d6,theta6);

% Q: Compute the transform from frame {0} to {1} and then find
% the end of link 1 position.
H01 = H01;
endOfLink1 = H01(1:3,4);

% Q: Compute the composite transform from frame {0} to {2} and then find
% the end of link 2 position.
H02 = H01*H12;
endOfLink2 = H02(1:3,4);

% Q: Compute the composite transform from frame {0} to {3} and then find
% the end of link 3 position.
H03 = H02*H23;
endOfLink3 = H03(1:3,4);

% Q: Compute the composite transform from frame {0} to {4} and then find
% the end of link 4 position.
H04 = H03*H34;
endOfLink4 = H04(1:3,4);

% Q: Compute the composite transform from frame {0} to {5} and then find
% the end of link 5 position.
H05 = H04*H45;
endOfLink5 = H05(1:3,4);

% Q: Compute the composite transform from frame {0} to {6} and then find
% the end of link 6 position.
H06 = H05*H56;
endOfLink6 = H06(1:3,4);

%Skip this part
if (nargin>1) 
    x_des = ee_des(1);
    y_des = ee_des(2);
    z_des = ee_des(3);
    %Plot the point where we want the end-effector
    plot3(x_des,y_des,z_des,'o','MarkerSize',10,'MarkerFaceColor','black');
    hold on; 
end

hold on;

%Draw line from origin to end of link 1
line([0 endOfLink1(1)],...
     [0 endOfLink1(2)],...
     [0 endOfLink1(3)],....
      'LineWidth',5,'Color','red');

%Draw line from end of link 1 to end of link 2
line([endOfLink1(1) endOfLink2(1)],...
     [endOfLink1(2) endOfLink2(2)],...
     [endOfLink1(3) endOfLink2(3)],...
     'LineWidth',5,'Color','blue');
 
%Draw line from end of link 2 to end of link 3
line([endOfLink2(1) endOfLink3(1)],...
     [endOfLink2(2) endOfLink3(2)],...
     [endOfLink2(3) endOfLink3(3)],...
     'LineWidth',5,'Color','green');
 
%Draw line from end of link 3 to end of link 4
line([endOfLink3(1) endOfLink4(1)],...
     [endOfLink3(2) endOfLink4(2)],...
     [endOfLink3(3) endOfLink4(3)],...
     'LineWidth',5,'Color','yellow');
 
%Draw line from end of link 4 to end of link 5
line([endOfLink4(1) endOfLink5(1)],...
     [endOfLink4(2) endOfLink5(2)],...
     [endOfLink4(3) endOfLink5(3)],...
     'LineWidth',5,'Color',[255,165,0]/255);
 
%Draw line from end of link 5 to end of link 6
line([endOfLink5(1) endOfLink6(1)],...
     [endOfLink5(2) endOfLink6(2)],...
     [endOfLink5(3) endOfLink6(3)],...
     'LineWidth',5,'Color','red');
 
xlabel('x');
ylabel('y');
zlabel('z');
grid on; 
view([148,10]); 
%These set the x and y limits for the axis (will need adjustment)
xlim([-2 2]); 
ylim([-2 2]);
zlim([-1 2.5]);
end
