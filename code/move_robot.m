%% Move robot
% Contains main loop to move the robot

flag_Kalman = 1; % use Kalman filter? 1-yes 0-no

% Initialize Kalman filter
Kalman_param.kl = 1e-3;
Kalman_param.kr = 1e-3;
Kalman_param.B  = 0.5;
Kalman_param.H = eye(3);
Kalman_param.F = eye(3);

x_hat_old = zeros(3,1); % prediction 

d = speed_init*time_step;
R_old  = zeros(3);
R_old = line_cov(R_old,Kalman_param.kl,Kalman_param.kr,Kalman_param.B,d);
P = inv(Kalman_param.H)*R_old*inv(Kalman_param.H');

% Initialize robot position
r_ref = [x_init y_init];

omega = 0;

tilt = 0;

% Initialize movement loop
error = 0;
done = 0;
loop_ind = 1;

while ~(error||done)  % movement loop
  
    % avoid collisions with obstacles using sonar
    [omega, tilt,sonar_data] = collision_avoidance(tilt,speed_init);
    
    % plot current sonar data
    h_sonar_plot = sonar_plot(handles.ax_sonar,sonar_data);
    % drawnow
    
    % move robot
    tic;
    pause(time_step);
    cur_time = toc;
    
    % localize the robot (incl. Kalman filter)
%     [r,theta,v,R_cur] = localize_robot(r_ref,x_hat_old,cur_time,omega,P,theta_ref,Kalman_param,R_old,flag_Kalman);
    [r,theta,v,R_cur] = localize_robot_test(r_ref,speed_init,x_hat_old,cur_time,omega,P,theta_init,Kalman_param,R_old,flag_Kalman);
    
    % update loop
    loop_ind = loop_ind + 1;
    R_old = R_cur;
end