%% run_mpc_cross5_hover.m — 5机 × 悬停（S6）
% 队形: cross5，十字，birth=[0,0,0; 0,3,0; 0,-3,0; 3,0,0; -3,0,0]，d*=3m（中心-臂）
% 运动: hover，原地悬停，T=30s
% MPC:  N=30，dt=0.05s，q_pos=4，q_vel=2，d_safe=1.5m
% 预期: PASS（form_err≈0m，min_sp=3m，五机十字悬停稳定；对应 S6 SITL 实验）
addpath(fileparts(mfilename('fullpath')));
scenario_run('cross5', 'hover');
