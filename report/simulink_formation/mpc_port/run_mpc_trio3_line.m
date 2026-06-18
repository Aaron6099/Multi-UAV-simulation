%% run_mpc_trio3_line.m — 3机 × 直线
% 队形: trio3，等边三角，birth=[3,0,0; -1.5,2.598,0; -1.5,-2.598,0]，d*≈5.196m
% 运动: line，北向直线，v=1.5m/s，d=20m，端点减速 a=0.5m/s²，T=50s
% MPC:  N=30，dt=0.05s，q_pos=4，q_vel=2，d_safe=1.5m
% 预期: REVIEW（track_err≈0.57m，共模速度滞后，form_err≈0.004m，队形完好）
addpath(fileparts(mfilename('fullpath')));
scenario_run('trio3', 'line');
