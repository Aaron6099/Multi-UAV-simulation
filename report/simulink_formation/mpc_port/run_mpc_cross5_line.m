%% run_mpc_cross5_line.m — 5机 × 直线（S4）
% 队形: cross5，十字，birth=[0,0,0; 0,3,0; 0,-3,0; 3,0,0; -3,0,0]，d*=3m（中心-臂）
% 运动: line，北向直线，v=1.0m/s（S4降速），d=20m，端点减速 a=0.5m/s²，T=50s
% MPC:  N=30，dt=0.05s，q_pos=4，q_vel=2，d_safe=1.5m
% 预期: PASS（track_err≈0.22m，低速消除大部分滞后；对应 S4 SITL 实验）
addpath(fileparts(mfilename('fullpath')));
scenario_run('cross5', 'line');
