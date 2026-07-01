%% run_mpc_grid9_hover.m — 9机 × 悬停
% 队形: grid9，3×3方阵，d*=3m
% 运动: hover，30s
% MPC:  N=30，dt=0.05s，q_pos=4，q_vel=2，d_safe=1.5m
% 预期: PASS（solve budget 15ms）
addpath(fileparts(mfilename('fullpath')));
scenario_run('grid9', 'hover');
