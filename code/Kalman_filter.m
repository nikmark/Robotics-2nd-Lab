function [x_hat_cur,z,P_cur,R_cur] = Kalman_filter(r_x,r_y,theta,r_ref,v_ref,x_hat_old,cur_time,omega,P,theta_ref,Kalman_param,R_old)
% Kalman_filter
%% Inputs:
% n_cyc: number of current cycle (index)
% x_hat_old: value of x_hat for previous cycle consists of [x,y,theta]
% cur_time: value of current time

    
    % calculating current x_hat (prediction)
    x_hat_cur(1:2) = Kalman_predict_pos(x_hat_old(1:2),v_ref,x_hat_old(3),degtorad(round(omega)),cur_time);
    x_hat_cur(3)   = x_hat_old(3) + degtorad(round(omega))*cur_time;
    x_hat_cur = x_hat_cur';
    
    % predictional measurement
    z_hat = Kalman_param.H*x_hat_cur;
    
    % update F
    F_cur = eye(3);
    if degtorad(round(omega)) == 0
        F_cur(1,3) = -v_ref*cur_time*sin(x_hat_old(3));
        F_cur(2,3) =  v_ref*cur_time*cos(x_hat_old(3));
    else if degtorad(round(omega)) > 0 
            F_cur(1,3) = v_ref/degtorad(round(omega))*( sin(x_hat_old(3) - pi/2) - sin(degtorad(round(omega))*cur_time + x_hat_old(3) - pi/2));
            F_cur(2,3) = v_ref/degtorad(round(omega))*(-cos(x_hat_old(3) - pi/2) + cos(degtorad(round(omega))*cur_time + x_hat_old(3) - pi/2));
        else  % omega < 0
            F_cur(1,3) = v_ref/degtorad(round(omega))*( sin(x_hat_old(3) - pi/2) + sin(degtorad(round(omega))*cur_time + x_hat_old(3) + pi/2));
            F_cur(2,3) = v_ref/degtorad(round(omega))*(-cos(x_hat_old(3) - pi/2) - cos(degtorad(round(omega))*cur_time + x_hat_old(3) + pi/2));    
        end
    end

    P_cur = F_cur*P*F_cur';
    
    % actualization of covariance matrix of odometry error
    d = v_ref*cur_time;
    if degtorad(round(omega)) == 0     
        R_cur    = line_cov(R_old,Kalman_param.kl,Kalman_param.kr,Kalman_param.B,d);
        R_xy = T(-x_hat_cur(3))*R_cur*T(-x_hat_cur(3))';
    else 
        dl   = ra_to_dl(v_ref/degtorad(round(omega)),degtorad(round(omega))*cur_time,Kalman_param.B);
        dr   = ra_to_dr(v_ref/degtorad(round(omega)),degtorad(round(omega))*cur_time,Kalman_param.B);
        R_cur    = arc_cov(R_old,dl,dr,Kalman_param.kl,Kalman_param.kr,Kalman_param.B);
        R_xy = T(-x_hat_cur(3))*R_cur*T(-x_hat_cur(3))';  
    end
        
    % Kalman factor
    K = P*Kalman_param.H'*inv(Kalman_param.H*P*Kalman_param.H' + R_xy);
    
    % measurement
    z = [ (([ cos(-theta_ref) sin(-theta_ref); -sin(-theta_ref) cos(-theta_ref) ]*[ r_x r_y ]')' + r_ref)'; theta + theta_ref ];
    if z(3) > pi 
        z(3) = 2*pi - z(3);
    end
    
    % measurement error
    e = z - z_hat;
    
    % correction
    x_hat_cur = x_hat_cur + K*e; 
    P_cur = P_cur - K*Kalman_param.H*P_cur;
    
end

    
    
    
    
    

    
    

    
    
    
  
    
    