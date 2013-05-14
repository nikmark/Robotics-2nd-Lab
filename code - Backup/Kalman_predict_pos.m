function x_pred = Kalman_predict_pos(x_old,v_ref,theta,omega,cur_time)
% calculate the prediction for the position of the robot


    if omega == 0
        x_pred = x_old + v_ref*cur_time*[ cos(theta); sin(theta) ];   
    else    
        % position of rotation center in robot reference frame
        c_rob = -v_ref/omega*[ cos(theta - pi/2); sin(theta - pi/2) ];

        % position of rotation center in world reference frame
        c = x_old + c_rob;

        % final robot position relative to the rotation center
        if omega > 0
            r_C_x = v_ref/omega*cos(omega*cur_time + theta - pi/2);
            r_C_y = v_ref/omega*sin(omega*cur_time + theta - pi/2);
            r_C = [r_C_x;r_C_y];        
        else   
            r_C_x = -v_ref/omega*cos(omega*cur_time + theta + pi/2);
            r_C_y = -v_ref/omega*sin(omega*cur_time + theta + pi/2);
            r_C = [r_C_x;r_C_y];        
        end

        % final position of robot in world reference frame
        x_pred = c + r_C;    
    end

end