function H = DH(a,alpha,d,theta)

H_z_theta = [cos(theta) -sin(theta) 0 0;
             sin(theta)  cos(theta) 0 0;
             0           0          1 0;
             0           0          0 1];
            
H_z_d = [1 0 0 0;
         0 1 0 0;
         0 0 1 d;
         0 0 0 1];
       
H_x_a = [1 0 0 a;
         0 1 0 0;
         0 0 1 0;
         0 0 0 1];
          
H_x_alpha = [1 0           0          0;
             0 cos(alpha) -sin(alpha) 0;
             0 sin(alpha)  cos(alpha) 0;
             0 0           0          1];

H = H_z_theta * H_z_d * H_x_a * H_x_alpha;
