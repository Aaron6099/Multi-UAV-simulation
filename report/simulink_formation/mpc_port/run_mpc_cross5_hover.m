%% run_mpc_cross5_hover.m — 真机部署对应件: cross5(5机) × hover
% S6 工况：5机十字编队悬停稳定性验证（Simulink 6DOF+PID 闭环）。
% 真机测 cross5 hover 前先跑本文件确认算法支撑。
addpath(fileparts(mfilename('fullpath')));
scenario_run('cross5', 'hover');
