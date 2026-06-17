%% run_mpc_cross5_line.m — 真机部署对应件: cross5(5机) × line
% S4 工况：5机十字编队北向直线 v=1.0 d=20m。
% 真机测 cross5 line 前先跑本文件确认算法支撑。
addpath(fileparts(mfilename('fullpath')));
scenario_run('cross5', 'line');
