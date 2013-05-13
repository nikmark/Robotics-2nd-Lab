global REC returnQ

GUI_DEFINE_MOTION_VARS

%% Variáveis auxiliares

GAMMA = pi/2:-pi/7:-pi/2;
S     = size(PATH);
TILT  = 0;

n_A     = 1;
n_C     = 1;
n_CICLE = 1;

%% Inicialização

M_i      = 1;
TARGET_i = M_i + (fix(STEP*v_REF/W_RES*PC_2) + 1);
M_F_i    = 1;

r      = r_REF;
THETA  = THETA_REF;
v      = [ cos(THETA) sin(THETA) ]*v_REF;
PHI    = 0;
OMEGA  = 0;
a_SEEK = [ 0 0 ];

v_SCALE = 5;
a_SCALE = .5;

% %% Inicialização do filtro de Kalman
% 
% R  = zeros(3);
% kl = 1e-3;
% kr = 1e-3;
% B  = .5;
% 
% H = eye(3);
% 
% F = eye(3);
% 
% % Previsão.
% xhat = zeros(3,1);
% 
% % Medição prevista.
% zhat = zeros(3,1);
% 
% % Medição.
% z = zeros(3,1);
% 
% % Erro da medição.
% e = zeros(3,1);

%% Inicialização do robot

pioneer_init(SP);
pause(3)

pioneer_set_controls(SP,round(v_REF*10^3),0);

pause(STEP)

d    = v_REF*STEP;
R    = line_cov(R,kl,kr,B,d);
R_xy = R;
P    = inv(H)*R*inv(H');

xhat(1:2,1) = (r + v*STEP)';
xhat(3,1)   = THETA + OMEGA*STEP;

r     = xhat(1:2,1)';
THETA = xhat(3,1);
v     = [ cos(THETA) sin(THETA) ]*v_REF;

COMPUTE_VARS

dr     = norm(r - PATH(M_i,:));
if M_i > 1
    dTHETA = radtodeg(GET_ANGLE(v,(PATH(M_i,:) - PATH(M_i - 1,:))));
else
    dTHETA = 0;
end

REC = [ r'; v'; a_SEEK'; PATH(TARGET_i,:)'; r_F'; PATH(M_F_i,:)'; radtodeg(PHI); OMEGA; radtodeg(THETA); STEP; z(1:2,1); radtodeg(z(3,1)); dr; dTHETA ];

n_CICLE = n_CICLE + 1;

if norm(PATH(end,:) - r) < PC_5;
    
    break
    
end

%% Guiamento

while 1
    
%     % Detecção de obstáculos.
%     if SONAR_on        
%         SONAR = pioneer_read_sonars();        
%         if isempty(SONAR)           
%             break          
%         end        
%     else        
%         SONAR = [];       
%     end   
    
%     if ~isempty(find(SONAR < PC_6*10^3,1)) && SONAR_on       
%         TILT = PC_7;       
%         [ MIN MIN_i ] = min(SONAR);       
%    
%         % Determinação do ângulo de correcção.
%         if MIN_i <= 4            
%             if MIN_i == 1              
%                 PHI = (GAMMA(2) - pi/2)/2;               
%             else                
%                 PHI = (GAMMA(MIN_i) - pi/2);                
%             end           
%         elseif MIN_i > 4          
%             if MIN_i == 8               
%                 PHI = (GAMMA(7) + pi/2)/2;               
%             else               
%                 PHI = (GAMMA(MIN_i) + pi/2);               
%             end           
%         end
%         PHI = PHI*PC_8;
%         
%         % Correcção da velocidade.        
%         OMEGA = radtodeg(PHI)/STEP;        
%         tic        
%         pioneer_set_controls(SP,round(v_REF*10^3),round(OMEGA));       
%         pause(STEP)       
%         n_A = n_A + 1;
%         
%     elseif ~TILT
%         
%         % Verificação da rota actual.
%         if abs((r_F - PATH(M_F_i,:))*ETA_n') > PC_3
%             
%             % Determinação do ângulo de correcção.
%             a_SEEK = ((PATH(TARGET_i,:) - r) - v)*PC_4;
%             v_SEEK = STEP*a_SEEK;
%             PHI    = GET_ANGLE(v,v + v_SEEK);
%             
%             % Correcção da velocidade.
%             OMEGA = radtodeg(PHI)/STEP;            
%             tic           
%             pioneer_set_controls(SP,round(v_REF*10^3),round(OMEGA));          
%             pause(STEP)           
%             n_C = n_C + 1;          
%         else          
%             a_SEEK = [ 0 0 ];
%             PHI    = 0;            
%             OMEGA = 0;          
%             tic         
%             pioneer_set_controls(SP,round(v_REF*10^3),0);          
%             pause(STEP)           
%         end
%         
%     else       
%         TILT = TILT - 1;        
%         a_SEEK = [ 0 0 ];
%         PHI    = 0;       
%         OMEGA = 0;       
%         tic       
%         pioneer_set_controls(SP,round(v_REF*10^3),0);       
%         pause(STEP)       
%     end
    
    PLOT
    
%     % Leitura da odometria.
%     [ r_x r_y THETA ] = BOT_ODOMETRY(pioneer_read_odometry());
    
    TICTOC = toc;
    
%     %% Filtro de Kalman
%     
%     % Previsão.
%     xhat(1:2,n_CICLE) = PREDICT_r(xhat(1:2,n_CICLE - 1),v_REF,xhat(3,n_CICLE - 1),degtorad(round(OMEGA)),TICTOC);
%     xhat(3,n_CICLE)   = xhat(3,n_CICLE - 1) + degtorad(round(OMEGA))*TICTOC;
%     
%     % Medição prevista.
%     zhat(:,n_CICLE) = H*xhat(:,n_CICLE);
%     
%     UPDATE_F
%     
%     P = F*P*F';
%     
%     % Actualização da matriz de covariância do erro da odometria.
%     d = v_REF*TICTOC;
%     if degtorad(round(OMEGA)) == 0
%         
%         R    = line_cov(R,kl,kr,B,d);
%         R_xy = T(-xhat(3,n_CICLE))*R*T(-xhat(3,n_CICLE))';
%         
%     else
%         
%         dl   = ra_to_dl(v_REF/degtorad(round(OMEGA)),degtorad(round(OMEGA))*TICTOC,B);
%         dr   = ra_to_dr(v_REF/degtorad(round(OMEGA)),degtorad(round(OMEGA))*TICTOC,B);
%         R    = arc_cov(R,dl,dr,kl,kr,B);
%         R_xy = T(-xhat(3,n_CICLE))*R*T(-xhat(3,n_CICLE))';
%         
%     end
%     
%     % Ganho de Kalman.
%     K = P*H'*inv(H*P*H' + R_xy);
%     
%     % Medição
%     z(:,n_CICLE) = [ (([ cos(-THETA_REF) sin(-THETA_REF); -sin(-THETA_REF) cos(-THETA_REF) ]*[ r_x r_y ]')' + r_REF)'; THETA + THETA_REF ];
%     
%     if z(3,n_CICLE) > pi       
%         z(3,n_CICLE) = 2*pi - z(3,n_CICLE);       
%     end
%     
%     % Erro da medição.
%     e(:,n_CICLE) = z(:,n_CICLE) - zhat(:,n_CICLE);
%     
%     % Correcção.
%     xhat(:,n_CICLE) = xhat(:,n_CICLE) + K*e(:,n_CICLE);
%     P = P - K*H*P;
    
    %% Guiamento
    
%     if KALMAN_on
%         
%         r     = xhat(1:2,n_CICLE)';
%         THETA = xhat(3,n_CICLE);
%         v     = [ cos(THETA) sin(THETA) ]*v_REF;
%         
%     else
%         
%         r     = z(1:2,n_CICLE)';
%         THETA = z(3,n_CICLE);
%         v     = [ cos(THETA) sin(THETA) ]*v_REF;
%         
%     end
    
    COMPUTE_VARS
    
    dr     = norm(r - PATH(M_i,:));
    if M_i > 1
        dTHETA = radtodeg(GET_ANGLE(v,(PATH(M_i,:) - PATH(M_i - 1,:))));
    else
        dTHETA = 0;
    end
    
    REC = [ REC [ r'; v'; a_SEEK'; PATH(TARGET_i,:)'; r_F'; PATH(M_F_i,:)'; radtodeg(PHI); degtorad(round(OMEGA)); radtodeg(THETA); TICTOC; z(1:2,n_CICLE); radtodeg(z(3,n_CICLE)); dr; dTHETA ] ];
    
    n_CICLE = n_CICLE + 1;
    
    if norm(PATH(end,:) - r) < PC_5;
        
        break
        
    end
    
end

disp('------------------------------');
disp(['#Obstacle avoidances:',num2str(n_A)]);
disp(['#Trajectory corrections:',num2str(n_C)]);

returnQ

if returnQ; zux = 'r.'; else zux = '.'; end
s = sprintf('.\\results\\REC_%s_%s%smat',num2str(v_REF),num2str(STEP),zux);
disp(['Path following simulation saved to ',s]);
save(s,'REC')

pioneer_set_controls(SP,0,0);
[BA LLS ] = wavread('steel.wav');
soundsc(BA,LLS);
pioneer_close(SP);

disp('------------------------------');
disp('Simulation running cycle info:');
TOTAL_CICLE = sum(REC(16,:))
MEAN_CICLE  = mean(REC(16,:))
STD_CICLE   = std(REC(16,:))
disp('------------------------------');