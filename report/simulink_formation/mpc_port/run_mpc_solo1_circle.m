%% run_mpc_solo1_circle.m — 1机 × 圆周
% 队形: solo1，birth=[0,0,0]，无邻居约束
% 运动: circle，R=10m，v=1.5m/s，周期≈42s，T=65s
% MPC:  N=30，dt=0.05s，q_pos=4，q_vel=2，d_safe=1.5m
% 预期: REVIEW（track_err≈1.18m，共模速度滞后；收敛需50s，队形无约束）
%       与 SITL solo1 circle 同判 REVIEW，交叉验证一致
addpath(fileparts(mfilename('fullpath')));
scenario_run('solo1', 'circle');
