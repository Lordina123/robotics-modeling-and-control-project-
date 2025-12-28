function [p_ref, dp_ref, ddp_ref] = generateInfinityTrajectory(x0, y0, A, B, T, dt)
    % Generates a figure-8 (infinity) trajectory and its derivatives
    % Inputs:
    %   x0, y0: center offset of trajectory
    %   A, B : amplitudes in x and y directions
    %   T    : period of the trajectory (seconds)
    %   dt   : timestep (seconds)
    % Outputs:
    %   p  : position vectors
    %   dp : velocity vectors
    %   ddp : acceleration vectors
    
    % Time vector
    t = 0:dt:T;
    w = 2*pi/T;  % angular frequency
    
    % Position 
    x = y0 + B * sin(w * t) .* cos(w * t);
    y = x0 + A * sin(w * t);
    
    % Velocity 
    dx =  B * w * cos(2*w*t); 
    dy = A * w * cos(w * t);
    
    % Acceleration 
    ddx = -2 * B * w^2 * sin(2*w*t);
    ddy = -A * w^2 * sin(w * t);

    p_ref   = [x; y];
    dp_ref  = [dx; dy];
    ddp_ref = [ddx; ddy];

end