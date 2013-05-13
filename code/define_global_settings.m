%% set some global variables (settings)
global m_point_window c_vel_pos d_max_pos_m_point c_acc_seek d_min_pos_last d_min_obst TILT c_acc_flee

% window size for searching M point
m_point_window = 64;

% proportional constant between velocity and future position
c_vel_pos = 5;

% maximal distance between future position and respective M point
d_max_pos_m_point = 0.1;

% acceleration coefficient for seeking
c_acc_seek = 0.1;

% minimal distance between current position and last position of path
d_min_pos_last = 0.25;

% minimal distance between current position and obstacle
d_min_obst = 0.5;

% tilt
TILT = 1;

% smoothing factor for fleeing angle
c_acc_flee = 0.5;