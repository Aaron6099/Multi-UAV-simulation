%% run_mpc_grid9_line.m — 9机 × 直线（S7）
% 队形: grid9，3×3方阵，birth=队形坐标，d*=3m
% 运动: line，北向直线，v=0.5m/s，d=12m，ready_hold=30s，T=85s
% MPC:  N=30，dt=0.05s，q_pos=4，q_vel=2，d_safe=1.5m
% 预期: PASS（track_err REVIEW级可接受；对应 S7 SITL 实验，solve budget 15ms）
addpath(fileparts(mfilename('fullpath')));
scenario_run('grid9', 'line');
