%% run_mpc_pair2_line.m — 真机部署对应件: pair2(2机) × line
% 完整移植 ROS mpc_node 的 MPC(双积分 OCP + GN-RTI + 碰撞/编队残差 + 邻居
% 预测轨迹交换)在 Simulink 6DOF+PID 被控对象上的闭环验证。
% 真机测 pair2 line 前先跑本文件确认算法支撑(VERDICT: PASS)。
addpath(fileparts(mfilename('fullpath')));
scenario_run('pair2', 'line');
