function out = consensus_swarm_step(in)
%CONSENSUS_SWARM_STEP 二阶一致性编队控制器（对照组，Interpreted MATLAB Fcn，50Hz）
%   签名同 mpc_swarm_step：
%     in  = [t; Xe1(3); Ve1(3); Xe2(3); Ve2(3); ...]   (世界 NED 真值)
%     out = [vx1 vy1 vz1 yaw1  vx2 ...]'                速度+偏航指令(NED)
%   领导-跟随二阶一致性 → 速度指令（δi = 队形偏移 cfg.offsets(i,:)）：
%     v_i = v0 − kp·avg_j[ (p_i−δi)−(p_j−δj) ] + 牵制[(p_i−δi)−p0]
%             − kv·avg_j[ v_i−v_j ]            + 牵制[ v_i−v0 ]
%   xy 走一致性律；z 沿用纯-P 定高（同 mpc_swarm_step）；yaw 沿用 center-yaw；
%   限幅 max_speed。邻接 = cfg.nbrs；邻居量测取瞬时值（无 MPC 的一拍延迟，
%   consensus 基线惯例），失谐项按 (邻居数+牵制) 归一化以使增益与队形规模无关。
global CONS_CFG
cfg = CONS_CFG;
n = cfg.n;

t = in(1);
[lp, lv, ~] = leader_state(t, cfg);
p0 = lp(:); v0 = lv(:);

% center-yaw：全员朝 leader 速度方向；悬停(lv≈0)保持上一拍（与 mpc_swarm_step 同款）
persistent last_yaw
if isempty(last_yaw), last_yaw = 0; end
if norm(lv(1:2)) > 0.05
    last_yaw = atan2(lv(2), lv(1));      % NED: atan2(east, north)
end
yaw_sp = last_yaw;

% 各机真值
P = zeros(3, n); V = zeros(3, n);
for i = 1:n
    P(:, i) = in(1 + 6*(i-1) + (1:3));
    V(:, i) = in(1 + 6*(i-1) + (4:6));
end
off = cfg.offsets;                       % n×3，δi

out = zeros(4*n, 1);
for i = 1:n
    pi_ = P(:, i); vi = V(:, i); di = off(i, :)';
    js  = cfg.nbrs{i}; M = numel(js); bi = cfg.pin(i);

    sp = zeros(3, 1); sv = zeros(3, 1);  % 邻居位置/速度失谐累加
    for m = 1:M
        j  = js(m); dj = off(j, :)';
        sp = sp + ((pi_ - di) - (P(:, j) - dj));
        sv = sv + (vi - V(:, j));
    end
    if M > 0, sp = sp / M; sv = sv / M; end   % 邻居项取均值（共模滞后 ~V/kp 与队形规模无关）
    sp = sp + bi * ((pi_ - di) - p0);          % 牵制项保原系数：拉向 leader+offset
    sv = sv + bi * (vi - v0);

    % 速度指令：xy 一致性律，z 纯-P 定高
    vel_sp = zeros(3, 1);
    vff = v0(1:2);                                   % leader 速度前馈
    if isfield(cfg, 'ff_leader_vel') && ~cfg.ff_leader_vel, vff = [0; 0]; end
    vel_sp(1:2) = vff - cfg.kp_cons * sp(1:2) - cfg.kv_cons * sv(1:2);
    ez = cfg.target_alt - pi_(3);
    vel_sp(3) = min(max(cfg.kp_z * ez, -cfg.max_climb), cfg.max_climb);
    vn = norm(vel_sp(1:2));
    if vn > cfg.max_speed, vel_sp(1:2) = vel_sp(1:2) * cfg.max_speed / vn; end
    if any(~isfinite(vel_sp)), vel_sp = zeros(3, 1); end

    out(4*(i-1) + (1:3)) = vel_sp;
    out(4*(i-1) + 4)     = yaw_sp;
end
end
