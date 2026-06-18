%% run_mpc_solo1_line.m — 1机 × 直线
% 队形: solo1，birth=[0,0,0]，无邻居约束
% 运动: line，北向直线，v=1.5m/s，d=20m，端点减速 a=0.5m/s²，T=50s
% MPC:  N=30，dt=0.05s，q_pos=4，q_vel=2，d_safe=1.5m
% 预期: REVIEW（track_err≈0.56m，速度内环无前馈滞后，无队形约束）
addpath(fileparts(mfilename('fullpath')));
scenario_run('solo1', 'line');
