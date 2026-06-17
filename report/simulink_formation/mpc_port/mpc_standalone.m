function res = mpc_standalone(formation, mode)
%MPC_STANDALONE 自包含多机 MPC 编队算法验证（纯 MATLAB，零外部依赖）
%
%   用法:  mpc_standalone('pair2',  'circle')   % 双机圆周
%          mpc_standalone('trio3',  'hover')    % 三机悬停
%          mpc_standalone('cross5', 'line')     % 五机直线
%          mpc_standalone                       % 默认 pair2 circle
%
%   支持:  solo1 / pair2 / trio3 / cross5  ×  hover / line / circle
%
%   算法:  完整移植 mpc_node.py
%          · 双积分 OCP (N=30, dt=0.05s)
%          · 单次 Gauss-Newton RTI (= acados SQP_RTI 语义)
%          · ADMM QP 求解（替代 acados HPIPM，无工具箱依赖）
%          · 碰撞/编队残差（GN 线性化）
%          · 邻居预测轨迹交换（一拍延迟 + 缺失推断降级）
%          · 速度合成：xy = MPC k=1 速度，z = 纯P
%
%   被控对象: 一阶速度响应（τ=0.3s）近似 PID 速度环 + 6DOF 内环。
%   注意: 数值与 Simulink 版略有差异（6DOF vs 一阶近似），判定逻辑完全一致。
%
%   所有算法局部函数（本文件底部）：
%     make_cfg / leader_state / precompute / solve_rti / admm

if nargin < 1, formation = 'pair2'; end
if nargin < 2, mode      = 'circle'; end

%% ── 配置 ────────────────────────────────────────────────────────────────
cfg    = make_cfg(formation, mode);
n      = cfg.n;
dt     = 0.02;        % 控制周期 50 Hz
tau_v  = 0.3;         % 速度环时间常数（一阶近似）

%% ── 预计算 QP 矩阵（每机一次，仅在 persistent 初始化时运行）────────────
P_qp = cell(n, 1);
for i = 1:n
    P_qp{i} = precompute(cfg, numel(cfg.nbrs{i}));
end

%% ── 初始状态（地面出生，z=0；高度控制驱动至 target_alt）──────────────
pos    = cfg.births';      % 3×n  世界系 NED
vel    = zeros(3, n);
vel_sp = zeros(3, n);      % 当前速度指令
zg     = cell(n, 1);       % MPC 热启动（空=冷启动）
peers  = cell(n, 1);       % 上一拍邻居预测轨迹

%% ── 仿真主循环 ───────────────────────────────────────────────────────────
Nsteps  = round(cfg.T / dt);
t_log   = (0:Nsteps-1)' * dt;
pos_log = zeros(Nsteps, 3, n);

fprintf('=== MPC standalone  %s × %s  (%d 机, T=%ds) ===\n', ...
    formation, mode, n, cfg.T);
tic
for k = 1:Nsteps
    t = (k-1) * dt;
    [lp, lv, la] = leader_state(t, cfg);

    newpeers = cell(n, 1);
    for i = 1:n
        x0 = [pos(:,i); vel(:,i)];

        %-- 参考轨迹：二阶 Taylor leader 预测 + 编队偏移
        %   对照 mpc_node._build_reference_trajectory
        xref = zeros(cfg.N+1, 6);
        off  = cfg.offsets(i,:);
        for kk = 0:cfg.N
            tk           = kk * cfg.mpc_dt;
            pk           = lp + lv*tk + 0.5*la*tk^2 + off;
            pk(3)        = cfg.target_alt;           % z 锁目标高度
            xref(kk+1,1:3) = pk;
            xref(kk+1,4:5) = lv(1:2) + la(1:2)*tk; % xy 速度前馈
        end

        %-- 邻居预测：上一拍广播（缺失→队形推断常值外推）
        js = cfg.nbrs{i};  M = numel(js);
        nb = [];
        if M > 0
            nb = zeros(M, cfg.N+1, 3);
            for m = 1:M
                j = js(m);
                if ~isempty(peers{j})
                    nb(m,:,:) = peers{j};
                else
                    infer = pos(:,i) + cfg.offsets(j,:)' - cfg.offsets(i,:)';
                    nb(m,:,:) = repmat(infer', cfg.N+1, 1);
                end
            end
        end

        %-- RTI 求解（单次 GN 迭代 + ADMM QP）
        [~, xpred, zg{i}] = solve_rti(x0, xref, nb, cfg.dstar{i}, zg{i}, P_qp{i}, cfg);
        newpeers{i} = xpred(:, 1:3);   % 发布本机预测轨迹

        %-- 速度合成：xy = MPC k=1 优化速度，z = 纯P 高度保持
        vs      = zeros(3,1);
        vs(1:2) = xpred(2, 4:5)';
        ez      = xref(1,3) - pos(3,i);
        vs(3)   = min(max(cfg.kp_z * ez, -cfg.max_climb), cfg.max_climb);
        vn      = norm(vs(1:2));
        if vn > cfg.max_speed, vs(1:2) = vs(1:2) * cfg.max_speed / vn; end
        if any(~isfinite(vs)), vs = zeros(3,1); end
        vel_sp(:,i) = vs;
    end
    peers = newpeers;   % 统一更新（一拍延迟，仿 ROS 异步广播）

    %-- 被控对象：一阶速度响应 → 积分得位置
    vel = vel + (vel_sp - vel) * (dt / tau_v);
    pos = pos + vel * dt;

    pos_log(k,:,:) = pos';
end
fprintf('  完成 %.1fs\n', toc);

%% ── 指标计算 ─────────────────────────────────────────────────────────────
% 参考轨迹（各机跟踪目标，xy 平面）
ref = zeros(Nsteps, 2, n);
lead_xy = zeros(Nsteps, 2);
for k = 1:Nsteps
    [lp,~,~] = leader_state(t_log(k), cfg);
    lead_xy(k,:) = lp(1:2);
    for i = 1:n
        ref(k,:,i) = lp(1:2) + cfg.offsets(i,1:2);
    end
end

% 跟踪误差（xy 平面距离）
track_err = squeeze(sqrt(sum((pos_log(:,1:2,:) - ref).^2, 2)));  % Nsteps×n
if n == 1, track_err = track_err(:); end

% 编队误差（邻居对间距偏差均值）
pairs = []; ds = [];
for i = 1:n
    for m = 1:numel(cfg.nbrs{i})
        j = cfg.nbrs{i}(m);
        if j > i
            pairs(end+1,:) = [i j]; %#ok<AGROW>
            ds(end+1)      = cfg.dstar{i}(m); %#ok<AGROW>
        end
    end
end
form_err = zeros(Nsteps,1);
min_sp   = inf(Nsteps,1);
for k = 1:Nsteps
    errs = [];
    for q = 1:size(pairs,1)
        d = norm(pos_log(k,1:2,pairs(q,1)) - pos_log(k,1:2,pairs(q,2)));
        errs(end+1) = abs(d - ds(q)); %#ok<AGROW>
        min_sp(k)   = min(min_sp(k), d);
    end
    if ~isempty(errs), form_err(k) = mean(errs); end
end

alt = squeeze(pos_log(:,3,:));
if n == 1, alt = alt(:); end

% 稳态窗口
steady  = t_log > cfg.t_start + 8;
m_form  = mean(form_err(steady));
m_track = mean(max(track_err(steady,:), [], 2));
m_alt   = mean(max(abs(alt(steady,:) - cfg.target_alt), [], 2));
m_minsp = min(min_sp(t_log > 8));

%% ── 三级判定（与 verify_formation.py 同阈值）────────────────────────────
core_ok = m_alt < 0.3 && (n == 1 || (m_form < 0.3 && m_minsp > cfg.d_safe));
if ~core_ok
    verdict = 'FAIL';
elseif m_track < 0.5
    verdict = 'PASS';
elseif m_track < 1.5
    verdict = 'REVIEW';   % 共模滞后：队形完好，整体均匀落后参考
else
    verdict = 'FAIL';
end

fprintf('  form_err=%.3fm  track_err=%.3fm  alt_err=%.3fm  min_sp=%.3fm\n', ...
    m_form, m_track, m_alt, m_minsp);
fprintf('  VERDICT: %s\n', verdict);

res = struct('formation',formation,'mode',mode,'form_err',m_form, ...
    'track_err',m_track,'alt_err',m_alt,'min_sp',m_minsp,'verdict',verdict);

%% ── 图（四格）────────────────────────────────────────────────────────────
figure('Position',[50 50 1280 720]);
cols = lines(n);

subplot(2,2,1); hold on; grid on; axis equal
for i = 1:n
    plot(pos_log(:,2,i), pos_log(:,1,i), '-', 'Color',cols(i,:),'LineWidth',1.2);
    plot(pos_log(end,2,i), pos_log(end,1,i), 'o', 'Color',cols(i,:), ...
        'MarkerFaceColor',cols(i,:), 'MarkerSize',7);
end
plot(lead_xy(:,2), lead_xy(:,1), 'k--', 'LineWidth',1);
xlabel('East [m]'); ylabel('North [m]');
title(sprintf('MPC %s %s — 俯视轨迹', formation, mode));
legend(arrayfun(@(i) sprintf('drone%d',i-1), 1:n,'UniformOutput',false), ...
    'leader','Location','best');

subplot(2,2,2); plot(t_log, form_err,'b','LineWidth',1.2); grid on
xlabel('t [s]'); ylabel('form\_err [m]'); title('编队误差（邻居对均值）');

subplot(2,2,3); plot(t_log, alt,'LineWidth',1.2); grid on
yline(cfg.target_alt,'k--','LineWidth',1);
xlabel('t [s]'); ylabel('z NED [m]'); title('高度');

subplot(2,2,4); hold on; grid on
if ~isempty(pairs)
    plot(t_log, min_sp,'b','LineWidth',1.2);
    yline(cfg.d_safe,'r--','d\_safe','LineWidth',1);
    ylabel('min spacing [m]'); title('最小间距');
else
    plot(t_log, track_err,'b','LineWidth',1.2);
    ylabel('track\_err [m]'); title('跟踪误差');
end
xlabel('t [s]');

sgtitle(sprintf('%s × %s  →  %s   (form=%.3fm  track=%.3fm  minsp=%.2fm)', ...
    formation, mode, verdict, m_form, m_track, m_minsp), 'FontWeight','bold');
end


%% ════════════════════════════════════════════════════════════════════════
%% 局部函数：所有算法在此，对照 mpc_node.py 注释已标行号
%% ════════════════════════════════════════════════════════════════════════

function cfg = make_cfg(formation, mode)
%MAKE_CFG 队形几何 + MPC 参数（对照 config/scenarios.yaml）
switch formation
    case 'solo1'
        births = [0 0 0];
        nbrs   = {[]};
    case 'pair2'
        births = [0 0 0; -3 0 0];
        nbrs   = {2, 1};
    case 'trio3'
        births = [3 0 0; -1.5 2.598 0; -1.5 -2.598 0];
        nbrs   = {[2 3], [1 3], [1 2]};
    case 'cross5'
        births = [0 0 0; 0 3 0; 0 -3 0; 3 0 0; -3 0 0];
        nbrs   = {[2 3 4 5], [1], [1], [1], [1]};
    otherwise
        error('未知 formation "%s"，支持: solo1/pair2/trio3/cross5', formation);
end

cfg.formation = formation;
cfg.mode      = mode;
cfg.n         = size(births, 1);
cfg.births    = births;
cfg.offsets   = births;   % offsets = birth（yaml 约定）
cfg.nbrs      = nbrs;

% 期望间距 d*（xy 平面，与 mpc_node desired_distances 同算法）
cfg.dstar = cell(cfg.n, 1);
for i = 1:cfg.n
    js = nbrs{i};
    cfg.dstar{i} = arrayfun(@(j) norm(births(i,1:2) - births(j,1:2)), js);
end

% MPC 参数（= scenarios.yaml defaults，逐项对照 mpc_node L107-181）
cfg.target_alt = -5.0;    % NED z（= 5m 高度）
cfg.max_speed  =  3.0;
cfg.max_climb  =  1.5;
cfg.max_accel  =  4.0;
cfg.N          = 30;      % mpc_horizon
cfg.mpc_dt     = 0.05;   % horizon 步长
cfg.q_pos      =  4.0;
cfg.q_vel      =  2.0;
cfg.r_acc      =  0.1;
cfg.q_term_s   =  2.0;   % terminal 权重倍率
cfg.d_safe     =  1.5;
cfg.w_coll     = 200.0;
cfg.w_form     =  0.5;
cfg.lm         =  1e-4;  % Levenberg-Marquardt 正则
cfg.kp_z       =  1.0;  % z 轴纯P高度保持增益

% 运动方式
cfg.t_start = 10.0;
switch mode
    case 'hover'
        cfg.T = 30;
    case 'line'
        cfg.lead_v = 1.5;  cfg.lead_d = 20.0;  cfg.lead_dec = 0.5;
        cfg.T = 50;
    case 'circle'
        cfg.lead_R = 10.0; cfg.lead_v = 1.5;
        cfg.T = 65;
    otherwise
        error('未知 mode "%s"，支持: hover/line/circle', mode);
end

% cross5 场景覆盖（= scenarios.yaml limits 字段）
if strcmp(formation, 'cross5')
    switch mode
        case 'line'
            cfg.lead_v = 1.0;
        case 'circle'
            cfg.lead_v   = 1.0;
            cfg.w_coll   = 500.0;
            cfg.t_start  = 30.0;
            cfg.T        = 100;
    end
end
end

% ─────────────────────────────────────────────────────────────────────────

function [p, v, a] = leader_state(t, cfg)
%LEADER_STATE leader 虚拟点位置/速度/加速度（NED，z 恒为 target_alt）
% 对照 leader_node：t_start 前悬停；line 带端点减速；circle 含向心加速度
alt = cfg.target_alt;
p = [0 0 alt]; v = [0 0 0]; a = [0 0 0];
tau = t - cfg.t_start;
if tau <= 0, return; end

switch cfg.mode
    case 'hover'
        % 原地不动

    case 'line'   % 北向(+x)，匀速 → 端点减速
        vmax = cfg.lead_v; d = cfg.lead_d; ad = cfg.lead_dec;
        s_dec = d - vmax^2 / (2*ad);
        t1    = s_dec / vmax;
        if tau <= t1
            s = vmax*tau;            vx = vmax;         ax = 0;
        else
            td = tau - t1;
            if td < vmax/ad
                s  = s_dec + vmax*td - 0.5*ad*td^2;
                vx = vmax - ad*td;   ax = -ad;
            else
                s = d; vx = 0; ax = 0;
            end
        end
        p(1) = s; v(1) = vx; a(1) = ax;

    case 'circle'  % 圆心在 (-R,0)，t=0 从原点出发，切向 +y 进入
        R = cfg.lead_R; vc = cfg.lead_v; om = vc / R;
        p(1) =  R*cos(om*tau) - R;
        p(2) =  R*sin(om*tau);
        v(1) = -vc*sin(om*tau);
        v(2) =  vc*cos(om*tau);
        a(1) = -vc*om*cos(om*tau);   % 向心加速度（前馈给 MPC 参考轨迹）
        a(2) = -vc*om*sin(om*tau);
end
end

% ─────────────────────────────────────────────────────────────────────────

function P = precompute(cfg, M)
%PRECOMPUTE 单机 MPC 常量结构（M = 邻居数）
% 双积分精确离散化 + 凝聚矩阵(xstack = Phi x0 + Gam u) + 约束矩阵
% 对照 mpc_node._setup_ocp (L107-181)
N = cfg.N; dt = cfg.mpc_dt; nx = 6; nu = 3;
P.N = N; P.nx = nx; P.nu = nu; P.M = M;

I3 = eye(3);
A  = [I3 dt*I3; zeros(3) I3];    % 双积分离散化 A
B  = [0.5*dt^2*I3; dt*I3];       % 双积分离散化 B
P.A = A; P.B = B;

P.nz  = nx*(N+1);
P.nuu = nu*N;
P.xidx = @(k) k*nx + (1:nx);
P.uidx = @(k) k*nu + (1:nu);

% 凝聚矩阵：xstack = Phi*x0 + Gam*ustack
Phi = zeros(P.nz, nx); Gam = zeros(P.nz, P.nuu);
Ak  = eye(nx);
Phi(1:nx,:) = Ak;
for k = 1:N
    Ak = A*Ak;
    Phi(P.xidx(k),:) = Ak;
    for j = 0:k-1
        Gam(P.xidx(k), P.uidx(j)) = A^(k-1-j)*B;
    end
end
P.Phi = Phi; P.Gam = Gam;

% 代价权重（stage / terminal）对照 mpc_node W 矩阵 L137-154
P.Q  = diag([cfg.q_pos*[1 1 1], cfg.q_vel*[1 1 1]]);
P.Qe = diag([cfg.q_pos*cfg.q_term_s*[1 1 1], cfg.q_vel*[1 1 1]]);
P.R  = cfg.r_acc * eye(3);
Hx   = zeros(P.nz, 1);
for k = 0:N-1, Hx(P.xidx(k)) = 2*diag(P.Q); end
Hx(P.xidx(N)) = 2*diag(P.Qe);
P.Hx_diag = Hx;
P.Hu = 2*kron(eye(N), P.R) + 2*cfg.lm*eye(P.nuu);   % 含 LM 正则

% 约束：u 箱 + 中间级速度界（对照 acados idxbx）
vel_rows = []; lv = []; uv = [];
for k = 1:N-1
    ix = P.xidx(k);
    vel_rows = [vel_rows, ix(4:6)]; %#ok<AGROW>
    lv = [lv; -cfg.max_speed; -cfg.max_speed; -cfg.max_climb]; %#ok<AGROW>
    uv = [uv;  cfg.max_speed;  cfg.max_speed;  cfg.max_climb]; %#ok<AGROW>
end
P.vel_rows = vel_rows;
P.Acon = [eye(P.nuu); Gam(vel_rows,:)];
P.lu   = -cfg.max_accel * ones(P.nuu, 1);
P.uu   =  cfg.max_accel * ones(P.nuu, 1);
P.lv   = lv; P.uv = uv;
end

% ─────────────────────────────────────────────────────────────────────────

function [u0, xpred, ws] = solve_rti(x0, xref, nb, dstar, ws, P, cfg)
%SOLVE_RTI 单次 Gauss-Newton SQP 迭代（= acados SQP_RTI 语义）
% x0   : 6×1 当前状态（世界 NED）
% xref : (N+1)×6 参考轨迹
% nb   : M×(N+1)×3 邻居预测轨迹（M=0 → []）
% dstar: 1×M 期望间距
% ws   : 热启动 struct（空=冷启动）
N  = P.N; nx = P.nx; M = P.M;

if isempty(ws)
    ws = struct('u', zeros(P.nuu,1), 'y', [], ...
                'xbar', repmat(x0', N+1, 1));
end

% x 堆叠上的 Hessian 对角 + 梯度（跟踪项，精确线性化）
Hx = spdiags(P.Hx_diag, 0, P.nz, P.nz);
fx = zeros(P.nz, 1);
for k = 0:N-1
    fx(P.xidx(k)) = -2 * P.Q * xref(k+1,:)';
end
fx(P.xidx(N)) = -2 * P.Qe * xref(N+1,:)';

% 碰撞/编队 GN 项（在上一拍预测 xbar 处线性化）
% 对照 mpc_node L130-135：碰撞残差 = √w_coll·max(0, d_safe-d)；
%                         编队残差 = √w_form·(d - d*)
swc = sqrt(cfg.w_coll); swf = sqrt(cfg.w_form);
for k = 0:N-1
    ix  = P.xidx(k); ixy = ix(1:2);
    xy  = ws.xbar(k+1, 1:2)';
    for m = 1:M
        nxy = squeeze(nb(m, k+1, 1:2));
        df  = xy - nxy;
        d   = sqrt(df'*df + 1e-6);
        % 编队残差（恒激活）
        g = swf * df / d;  r = swf * (d - dstar(m));
        Hx(ixy,ixy) = Hx(ixy,ixy) + 2*(g*g');
        fx(ixy)      = fx(ixy)      + 2*(r - g'*xy)*g;
        % 碰撞残差（仅 d < d_safe 时激活）
        if d < cfg.d_safe
            g = -swc * df / d;  r = swc * (cfg.d_safe - d);
            Hx(ixy,ixy) = Hx(ixy,ixy) + 2*(g*g');
            fx(ixy)      = fx(ixy)      + 2*(r - g'*xy)*g;
        end
    end
end

% 凝聚到 u 空间（消去 x）
phix = P.Phi * x0;
HxG  = Hx * P.Gam;
Pu   = P.Gam' * HxG + P.Hu;
Pu   = (Pu + Pu') / 2;              % 对称化消浮点误差
qu   = P.Gam' * (fx + Hx * phix);

l = [P.lu; P.lv - phix(P.vel_rows)];
u = [P.uu; P.uv - phix(P.vel_rows)];

% ADMM QP 求解（替代 acados HPIPM，无工具箱依赖）
[usol, ydual] = admm(Pu, qu, P.Acon, l, u, ws.u, ws.y);
if any(~isfinite(usol)), usol = ws.u; end

xstack = phix + P.Gam * usol;
xpred  = reshape(xstack, nx, N+1)';
u0     = usol(1:P.nu);
ws.u   = usol; ws.y = ydual; ws.xbar = xpred;
end

% ─────────────────────────────────────────────────────────────────────────

function [u, y] = admm(P, q, A, l, ub, u0, y0)
%ADMM OSQP 风格 ADMM 求解  min ½uᵀPu + qᵀu  s.t. l ≤ Au ≤ ub
% 小规模稠密专用（本仓 MPC：90 变量 / ~180 约束）。无工具箱依赖。
rho = 10.0; sigma = 1e-6; iters = 200; tol = 1e-4;
nu  = numel(q);
if isempty(u0), u0 = zeros(nu,1); end
if isempty(y0), y0 = zeros(size(A,1),1); end
u = u0; y = y0;
z = min(max(A*u, l), ub);
K = chol(P + sigma*eye(nu) + rho*(A'*A), 'lower');
for it = 1:iters
    rhs   = sigma*u - q + A'*(rho*z - y);
    u     = K' \ (K \ rhs);
    Au    = A*u;
    z_new = min(max(Au + y/rho, l), ub);
    y     = y + rho*(Au - z_new);
    if norm(Au-z_new,inf) < tol && rho*norm(z_new-z,inf) < tol, break; end
    z = z_new;
end
end
