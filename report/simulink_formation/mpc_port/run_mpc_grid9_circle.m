%% run_mpc_grid9_circle.m — 9机 × 圆周（S8）
% 队形: grid9，3×3方阵，d*=3m
% 运动: circle，R=10m，v=0.5m/s，ready_hold=30s，T=165s（约1整圈）
% MPC:  N=30，dt=0.05s，q_pos=4，q_vel=2，d_safe=1.5m
% 预期: PASS（对应 S8 SITL 实验，solve budget 15ms）
addpath(fileparts(mfilename('fullpath')));
scenario_run('grid9', 'circle');
