%% run_mpc_trio3_hover.m — 3机 × 悬停
% 队形: trio3，等边三角，birth=[3,0,0; -1.5,2.598,0; -1.5,-2.598,0]，d*≈5.196m
% 运动: hover，原地悬停，T=30s
% MPC:  N=30，dt=0.05s，q_pos=4，q_vel=2，d_safe=1.5m
% 预期: PASS（form_err≈0m，min_sp≈5.196m，三机悬停稳定）
addpath(fileparts(mfilename('fullpath')));
scenario_run('trio3', 'hover');
