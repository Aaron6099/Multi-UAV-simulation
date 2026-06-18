%% run_mpc_cross5_circle.m — 5机 × 圆周（S5）
% 队形: cross5，十字，birth=[0,0,0; 0,3,0; 0,-3,0; 3,0,0; -3,0,0]，d*=3m（中心-臂）
% 运动: circle，R=10m，v=1.0m/s，周期≈62.8s，t_start=30s（5机成型缓冲），T=93s
% MPC:  N=30，dt=0.05s，q_pos=4，q_vel=2，d_safe=1.5m，w_coll=500（防中心-臂碰）
% 预期: PASS（track_err≈0.13m，低速圆周稳定性最优；对应 S5/S13 SITL 38min 实验）
addpath(fileparts(mfilename('fullpath')));
scenario_run('cross5', 'circle');
