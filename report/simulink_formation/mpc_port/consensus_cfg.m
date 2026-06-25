function cfg = consensus_cfg(formation, mode, ff)
%CONSENSUS_CFG 二阶一致性对照组配置 = formation_cfg + 一致性增益
%   几何/邻接/leader 轨迹/限幅/T 全部照搬 formation_cfg，保证与 MPC 同口径对比，
%   只追加一致性控制律所需增益（kp/kv/牵制）。改队形/工况仍改 formation_cfg。
%   ff (可选,默认 true): 速度指令是否含 leader 速度前馈 v0。
%     true  = 带前馈（跟踪近乎无滞后；track_err 与 MPC 不对等，见讨论）
%     false = 公平基线（去前馈，纯位置一致性 → 暴露 V/kp 共模滞后，与 MPC 同信息结构）
if nargin < 3, ff = true; end
cfg = formation_cfg(formation, mode);
cfg.ctrl = 'consensus';

% ── 二阶一致性增益（速度指令型 PD-consensus）──────────────────────────────
cfg.kp_cons = 1.0;              % 位置失谐增益（越大跟踪越紧、越易激进）
cfg.kv_cons = 0.5;              % 速度失谐增益（"二阶"阻尼项：抑制相对速度）
cfg.pin     = ones(cfg.n, 1);  % 牵制 b_i：每机都能观测虚拟 leader（信息量同 MPC）
cfg.ff_leader_vel = ff;        % leader 速度前馈开关（公平基线 ablation 设 false）
cfg.suffix  = '';  if ~ff, cfg.suffix = '_noff'; end
end
