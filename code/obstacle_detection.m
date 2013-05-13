function [ output_args ] = obstacle_detection( input_args )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Detecção de obstáculos.
    if SONAR_on        
        SONAR = pioneer_read_sonars();        
        if isempty(SONAR)           
            break          
        end        
    else        
        SONAR = [];       
    end   
    if ~isempty(find(SONAR < PC_6*10^3,1)) && SONAR_on       
        TILT = PC_7;       
        [ MIN MIN_i ] = min(SONAR);       
   
        % Determinação do ângulo de correcção.
        if MIN_i <= 4            
            if MIN_i == 1              
                PHI = (GAMMA(2) - pi/2)/2;               
            else                
                PHI = (GAMMA(MIN_i) - pi/2);                
            end           
        elseif MIN_i > 4          
            if MIN_i == 8               
                PHI = (GAMMA(7) + pi/2)/2;               
            else               
                PHI = (GAMMA(MIN_i) + pi/2);               
            end           
        end
        PHI = PHI*PC_8;
        
        % Correcção da velocidade.        
        OMEGA = radtodeg(PHI)/STEP;        
        tic        
        pioneer_set_controls(SP,round(v_REF*10^3),round(OMEGA));       
        pause(STEP)       
        n_A = n_A + 1;
        
         elseif ~TILT
        
        % Verificação da rota actual.
        if abs((r_F - PATH(M_F_i,:))*ETA_n') > PC_3
            
            % Determinação do ângulo de correcção.
            a_SEEK = ((PATH(TARGET_i,:) - r) - v)*PC_4;
            v_SEEK = STEP*a_SEEK;
            PHI    = GET_ANGLE(v,v + v_SEEK);
            
            % Correcção da velocidade.
            OMEGA = radtodeg(PHI)/STEP;            
            tic           
            pioneer_set_controls(SP,round(v_REF*10^3),round(OMEGA));          
            pause(STEP)           
            n_C = n_C + 1;          
        else          
            a_SEEK = [ 0 0 ];
            PHI    = 0;            
            OMEGA = 0;          
            tic         
            pioneer_set_controls(SP,round(v_REF*10^3),0);          
            pause(STEP)           
        end
        
    else       
        TILT = TILT - 1;        
        a_SEEK = [ 0 0 ];
        PHI    = 0;       
        OMEGA = 0;       
        tic       
        pioneer_set_controls(SP,round(v_REF*10^3),0);       
        pause(STEP)       
    end
end

