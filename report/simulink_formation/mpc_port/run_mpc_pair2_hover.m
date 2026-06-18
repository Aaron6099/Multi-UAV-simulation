%% run_mpc_pair2_hover.m — 2机 × 悬停
% 队形: pair2，birth=[0,0,0; -3,0,0]，d*=3m（南北纵列）
% 运动: hover，原地悬停，T=30s
% MPC:  N=30，dt=0.05s，q_pos=4，q_vel=2，d_safe=1.5m
% 预期: PASS（form_err≈0m，min_sp=3m，双机悬停稳定）
addpath(fileparts(mfilename('fullpath')));
scenario_run('pair2', 'hover');
