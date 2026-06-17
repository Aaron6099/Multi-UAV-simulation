%% run_mpc_cross5_circle.m — 真机部署对应件: cross5(5机) × circle
% S5 工况：5机十字编队 R=10m v=1.0 圆周，ready_hold=30s，w_coll=500。
% 真机测 cross5 circle 前先跑本文件确认算法支撑。
addpath(fileparts(mfilename('fullpath')));
scenario_run('cross5', 'circle');
