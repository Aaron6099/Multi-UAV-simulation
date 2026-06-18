%% run_mpc_solo1_hover.m — 1机 × 悬停
% 队形: solo1，birth=[0,0,0]，无邻居约束
% 运动: hover，原地悬停，T=30s
% MPC:  N=30，dt=0.05s，q_pos=4，q_vel=2，d_safe=1.5m
% 预期: PASS（track_err≈0m，alt_err<0.001m，悬停稳定）
addpath(fileparts(mfilename('fullpath')));
scenario_run('solo1', 'hover');
