function [ omega, tilt ] = collision_avoidance(tilt,v_ref)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

global d_min_obst TILT c_acc_flee SP timestep PATH

% auxiliry variables
gamma = pi/2:-pi/7:-pi/2;

% detection of obstacles  
sonar_data = pioneer_read_sonars();        
      
    
    if ~isempty(find(sonar_data < d_min_obst*10^3,1)) %&& SONAR_on       
        tilt = TILT;       
        [sonar_data_min ,sonar_data_min_ind ] = min(sonar_data);       
   
        % determination of correction angle
        if sonar_data_min_ind <= 4            
            if sonar_data_min_ind == 1              
                phi = (gamma(2) - pi/2)/2;               
            else                
                phi = (gamma(sonar_data_min_ind) - pi/2);                
            end           
        elseif sonar_data_min_ind > 4          
            if sonar_data_min_ind == 8               
                phi = (gamma(7) + pi/2)/2;               
            else               
                phi = (gamma(sonar_data_min_ind) + pi/2);               
            end           
        end
        phi = phi*c_acc_flee;
        
        % correction of velocity        
        omega = radtodeg(phi)/timestep;     
        tic        
        pioneer_set_controls(SP,round(v_ref*10^3),round(omega));       
        pause(timestep)       
        n_A = n_A + 1;
        
        elseif ~tilt
        
        % verification of current route
        if abs((r_F - PATH(M_F_i,:))*ETA_n') > PC_3
            
            % determination of correction angle
            a_SEEK = ((PATH(TARGET_i,:) - r) - v)*PC_4;
            v_SEEK = timestep*a_SEEK;
            phi    = GET_ANGLE(v,v + v_SEEK);
            
            % Correcção da velocidade.
            omega = radtodeg(phi)/timestep;            
            tic           
            pioneer_set_controls(SP,round(v_ref*10^3),round(omega));          
            pause(timestep)           
            n_C = n_C + 1;          
        else          
            a_SEEK = [ 0 0 ];
            phi    = 0;            
            omega = 0;          
            tic         
            pioneer_set_controls(SP,round(v_ref*10^3),0);          
            pause(timestep)           
        end
        
    else       
        tilt = tilt - 1;        
        a_SEEK = [ 0 0 ];
        phi    = 0;       
        omega = 0;       
        tic       
        pioneer_set_controls(SP,round(v_ref*10^3),0);       
        pause(timestep)       
    end
end

