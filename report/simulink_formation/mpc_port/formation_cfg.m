function cfg = formation_cfg(formation, mode)
%FORMATION_CFG 队形+运动方式 → 完整场景配置（与 config/scenarios.yaml 同真值）
%   formation: 'solo1' | 'pair2' | 'trio3'
%   mode:      'hover' | 'line'  | 'circle'
% 几何(NED)与 MPC 参数逐项对照 scenarios.yaml defaults + formations，
% 改 yaml 后请同步本文件（Simulink 侧不解析 yaml，避免依赖）。

% ── 队形几何（= scenarios.yaml formations.*）────────────────────────────
switch formation
    case 'solo1'
        births = [0 0 0];
        nbrs   = {[]};
    case 'pair2'
        births = [0 0 0; -3 0 0];
        nbrs   = {2, 1};               % MATLAB 1-based
    case 'trio3'
        births = [3 0 0; -1.5 2.598 0; -1.5 -2.598 0];
        nbrs   = {[2 3], [1 3], [1 2]};
    otherwise
        error('未知 formation "%s"', formation);
end
cfg.formation = formation;
cfg.mode      = mode;
cfg.n         = size(births, 1);
cfg.births    = births;
cfg.offsets   = births;                % offsets 省略 = 同 birth（yaml 约定）
cfg.nbrs      = nbrs;

% 期望间距 d*（xy 平面，与 mpc_node desired_distances 同算法）
cfg.dstar = cell(cfg.n, 1);
for i = 1:cfg.n
    js = nbrs{i};
    cfg.dstar{i} = arrayfun(@(j) norm(cfg.offsets(i,1:2) - cfg.offsets(j,1:2)), js);
end

% ── MPC 参数（= scenarios.yaml defaults）────────────────────────────────
cfg.target_alt = -5.0;
cfg.max_speed  =  3.0;
cfg.max_climb  =  1.5;
cfg.max_accel  =  4.0;
cfg.control_dt =  0.02;        % control_hz = 50
cfg.N          = 30;           % mpc_horizon
cfg.mpc_dt     = 0.05;
cfg.q_pos      = 4.0;
cfg.q_vel      = 2.0;
cfg.r_acc      = 0.1;
cfg.q_term_s   = 2.0;
cfg.d_safe     = 1.5;
cfg.w_coll     = 200.0;
cfg.w_form     = 0.5;
cfg.lm         = 1e-4;         % acados levenberg_marquardt
cfg.kp_z       = 1.0;          % 节点 z 轴纯 P 高度保持增益

% ── leader 轨迹（hover/line/circle，对应 leader_node 运动方式）──────────
cfg.t_start = 10.0;            % 爬升+成型阶段后 leader 才开动
switch mode
    case 'hover'
        cfg.T = 30;
    case 'line'                % 北向直线 v=1.5, d=20, 端点减速 a=0.5
        cfg.lead_v = 1.5; cfg.lead_d = 20.0; cfg.lead_dec = 0.5;
        cfg.T = 50;
    case 'circle'              % R=10, v=1.5（≈一整圈 42s）
        cfg.lead_R = 10.0; cfg.lead_v = 1.5;
        cfg.T = 65;
    otherwise
        error('未知 mode "%s"', mode);
end
end
