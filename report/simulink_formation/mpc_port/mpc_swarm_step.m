function out = mpc_swarm_step(in)
%MPC_SWARM_STEP Simulink 内的多机 MPC 控制器（Interpreted MATLAB Fcn，50Hz）
%   in  = [t; Xe1(3); Ve1(3); Xe2(3); Ve2(3); ...]   (世界 NED 真值)
%   out = [vx1 vy1 vz1 yaw1  vx2 vy2 vz2 yaw2 ...]'  速度+偏航指令(NED)
% 逐行对照 mpc_node.control_loop：参考轨迹二阶预测 → 邻居预测轨迹(上一拍,
% 缺失时队形推断降级) → RTI 求解 → 速度合成(xy=MPC 预测 k=1 速度,
% z=纯 P 高度保持) → 限幅。场景配置经 global MPC_CFG 注入。
global MPC_CFG
cfg = MPC_CFG;
n = cfg.n;

persistent P zg peers
if isempty(P)
    P = cell(n,1); zg = cell(n,1); peers = cell(n,1);
    for i = 1:n
        P{i} = mpc_precompute(cfg, numel(cfg.nbrs{i}));
    end
end

t = in(1);
[lp, lv, la] = leader_state(t, cfg);

% center-yaw：全员统一朝 leader 速度方向；悬停(lv≈0)保持上一拍
persistent last_yaw
if isempty(last_yaw), last_yaw = 0; end
lv_norm = norm(lv(1:2));
if lv_norm > 0.05
    last_yaw = atan2(lv(2), lv(1));   % NED: atan2(east, north)
end
yaw_sp = last_yaw;

out = zeros(4*n, 1);
newpeers = cell(n,1);
N1 = cfg.N + 1;
for i = 1:n
    pos = in(1 + 6*(i-1) + (1:3));
    vel = in(1 + 6*(i-1) + (4:6));
    x0  = [pos; vel];

    % 参考轨迹（mpc_node._build_reference_trajectory：二阶 leader 预测 + 偏移）
    xref = zeros(N1, 6);
    off = cfg.offsets(i, :);
    for k = 0:cfg.N
        tk = k * cfg.mpc_dt;
        pk = lp + lv*tk + 0.5*la*tk^2 + off;
        pk(3) = cfg.target_alt;
        xref(k+1, 1:3) = pk;
        xref(k+1, 4:5) = lv(1:2) + la(1:2)*tk;
    end

    % 邻居预测（上一拍发布；缺失 → 队形推断常值外推，与节点降级路径一致）
    js = cfg.nbrs{i}; M = numel(js);
    if M > 0
        nb = zeros(M, N1, 3);
        for m = 1:M
            j = js(m);
            if ~isempty(peers{j})
                nb(m, :, :) = peers{j};
            else
                infer = pos' + cfg.offsets(j,:) - off;
                nb(m, :, :) = repmat(infer, N1, 1);
            end
        end
    else
        nb = [];
    end

    [~, xpred, zg{i}] = mpc_solve_rti(x0, xref, nb, cfg.dstar{i}, zg{i}, P{i}, cfg);
    newpeers{i} = xpred(:, 1:3);

    % 速度合成（节点速度控制段同款）
    vel_sp = zeros(3,1);
    vel_sp(1:2) = xpred(2, 4:5)';                            % xy: MPC 优化速度
    ez = xref(1,3) - pos(3);
    vel_sp(3) = min(max(cfg.kp_z * ez, -cfg.max_climb), cfg.max_climb);  % z: 纯 P
    vn = norm(vel_sp(1:2));
    if vn > cfg.max_speed, vel_sp(1:2) = vel_sp(1:2) * cfg.max_speed / vn; end
    if any(~isfinite(vel_sp)), vel_sp = zeros(3,1); end

    out(4*(i-1) + (1:3)) = vel_sp;
    out(4*(i-1) + 4)     = yaw_sp;
end
peers = newpeers;    % 循环后统一发布 = 全员一拍延迟（仿 ROS 异步交换）
end
