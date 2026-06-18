%% run_mpc_trio3_circle.m — 3机 × 圆周
% 队形: trio3，等边三角，birth=[3,0,0; -1.5,2.598,0; -1.5,-2.598,0]，d*≈5.196m
% 运动: circle，R=10m，v=1.5m/s，周期≈42s，T=65s
% MPC:  N=30，dt=0.05s，q_pos=4，q_vel=2，d_safe=1.5m
% 预期: REVIEW（track_err≈1.18m，共模速度滞后，form_err≈0.020m，队形完好）
%       SITL trio3 circle 1.129m vs Simulink 1.185m，双栈交叉印证同判 REVIEW
addpath(fileparts(mfilename('fullpath')));
scenario_run('trio3', 'circle');
