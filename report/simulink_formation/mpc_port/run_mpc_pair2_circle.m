%% run_mpc_pair2_circle.m — 2机 × 圆周
% 队形: pair2，birth=[0,0,0; -3,0,0]，d*=3m（南北纵列）
% 运动: circle，R=10m，v=1.5m/s，周期≈42s，T=65s
% MPC:  N=30，dt=0.05s，q_pos=4，q_vel=2，d_safe=1.5m
% 预期: REVIEW（track_err≈1.18m，共模速度滞后，form_err≈0m，队形完好）
addpath(fileparts(mfilename('fullpath')));
scenario_run('pair2', 'circle');
