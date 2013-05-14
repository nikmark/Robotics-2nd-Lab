%% Robot localization
function [r,theta,v,R_cur] = localize_robot(r_ref,v_ref,x_hat_old,cur_time,omega,P,theta_ref,Kalman_param,R_old,flag_Kalman) 

% read odometry data from robot
[r_x,r_y,theta] = pioneer_read_odometry();
r_x = r_x/1e3;
r_y = r_y/1e3;
theta = theta*pi/2048;


% apply Kalman filter
if flag_Kalman
[x_hat_cur,z,P_cur,R_cur] = Kalman_filter(r_x,r_y,theta,r_ref,v_ref,x_hat_old,cur_time,omega,P,theta_ref,Kalman_param,R_old);
end

% get new position, orientation and velocity
if flag_Kalman        
        r     = x_hat_cur(1:2)';
        theta = x_hat_cur(3);
        v     = [ cos(theta) sin(theta) ]*v_ref;
else
        r     = z(1:2)';
        theta = z(3);
        v     = [ cos(theta) sin(theta) ]*v_ref;    
end
