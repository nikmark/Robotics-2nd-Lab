%% Move robot
% Contains main loop to move the robot


% Initialize Kalman filter
R_old  = zeros(3);
Kalman_param.kl = 1e-3;
Kalman_param.kr = 1e-3;
Kalman_param.B  = 0.5;
Kalman_param.H = eye(3);
Kalman_param.F = eye(3);


% initialize robot position


% initialize movement loop
error = 0;
done = 0;
loop_ind = 1;

while ~(error||done)  % movement loop
  
    % obstacle detection
    obstacle_detection();
    
    % move robot
    
    % localize the robot (incl. Kalman filter)
    [r,theta,v,R_cur] = localize_robot(Kalman_param,R_old);
    
    % update loop
    loop_ind = loop_ind + 1;
    R_old = R_cur;
end