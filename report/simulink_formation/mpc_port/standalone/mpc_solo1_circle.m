function res = mpc_solo1_circle()
%MPC_SOLO1_CIRCLE  自包含 MPC 验证：单机 × 圆周
%
% 直接运行此文件，无需任何外部依赖（不需要 .slx / 工具箱 / acados）。
%
% ── 场景 ──────────────────────────────────────────────────────────────
% 队形 : solo1（单机，1 机）
% 运动 : circle（圆周）
% 时长 : T = 65 s，成型等待 t_start = 10 s
% cfg.lead_R = 10.0;      % 圆半径 m
% cfg.lead_v = 1.5;      % 圆周速度 m/s  → 周期≈42s
%
% 预期结果（对照 results.csv）：
% REVIEW = 共模速度滞后（队形完好、间距安全、整体均匀落后参考）= 非编队缺陷
%
% ── 被控对象 ──────────────────────────────────────────────────────────
% 一阶速度响应（τ=0.3s）近似 PID 速度环 + 6DOF 内环。
% 数值与 Simulink 6DOF 版略有差异，判定逻辑完全一致。
%
% ── 输出 ──────────────────────────────────────────────────────────────
% 控制台打印 form_err / track_err / alt_err / min_sp / VERDICT
% 弹出四格图：俯视轨迹 / 编队误差 / 高度 / 最小间距

%% ── 配置 ─────────────────────────────────────────────────────────────
    cfg.formation = 'solo1';
    cfg.mode      = 'circle';
    cfg.n         = 1;
    cfg.births    = [0 0 0];
    cfg.offsets   = cfg.births;
    cfg.nbrs      = {{[]}};

    % 期望间距 d*（xy 平面）
    cfg.dstar = cell(cfg.n, 1);
    for i = 1:cfg.n
        js = cfg.nbrs{i};
        cfg.dstar{i} = arrayfun(@(j) norm(cfg.offsets(i,1:2)-cfg.offsets(j,1:2)), js);
    end

    % MPC 参数（= scenarios.yaml defaults）
    cfg.target_alt = -5.0;
    cfg.max_speed  =  3.0;
    cfg.max_climb  =  1.5;
    cfg.max_accel  =  4.0;
    cfg.N          = 30;
    cfg.mpc_dt     = 0.05;
    cfg.q_pos      =  4.0;
    cfg.q_vel      =  2.0;
    cfg.r_acc      =  0.1;
    cfg.q_term_s   =  2.0;
    cfg.d_safe     =  1.5;
    cfg.w_coll     = 200.0;
    cfg.w_form     =  0.5;
    cfg.lm         =  1e-4;
    cfg.kp_z       =  1.0;
    cfg.t_start    = 10.0;
    cfg.T          = 65;
    cfg.lead_R = 10.0;
    cfg.lead_v = 1.5;

%% ── 初始状态 ─────────────────────────────────────────────────────────
dt    = 0.02;     % 控制周期 50 Hz
tau_v = 0.3;      % 速度环时间常数
n     = cfg.n;

pos    = zeros(3, n);
    pos(:,1) = [0 0 0]';
vel    = zeros(3, n);
vel_sp = zeros(3, n);

%% ── 预计算 QP 矩阵 ────────────────────────────────────────────────────
P_qp = cell(n, 1);
for i = 1:n
    P_qp{i} = precompute(cfg, numel(cfg.nbrs{i}));
end
zg    = cell(n, 1);   % MPC 热启动
peers = cell(n, 1);   % 邻居预测轨迹（上一拍）

%% ── 仿真主循环 ────────────────────────────────────────────────────────
Nsteps  = round(cfg.T / dt);
t_log   = (0:Nsteps-1)' * dt;
pos_log = zeros(Nsteps, 3, n);

fprintf('=== solo1 × circle  （1 机, T=65s）===\n');
tic
for k = 1:Nsteps
    t = (k-1) * dt;
    [lp, lv, la] = leader_state(t, cfg);
    newpeers = cell(n, 1);

    for i = 1:n
        x0  = [pos(:,i); vel(:,i)];
        off = cfg.offsets(i,:);

        % 参考轨迹：二阶 Taylor leader 预测 + 编队偏移
        xref = zeros(cfg.N+1, 6);
        for kk = 0:cfg.N
            tk = kk * cfg.mpc_dt;
            pk = lp + lv*tk + 0.5*la*tk^2 + off;
            pk(3) = cfg.target_alt;
            xref(kk+1,1:3) = pk;
            xref(kk+1,4:5) = lv(1:2) + la(1:2)*tk;
        end

        % 邻居预测（上一拍；缺失 → 队形推断降级）
        js = cfg.nbrs{i}; M = numel(js); nb = [];
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

        % RTI 求解（GN 单步 + ADMM QP）
        [~, xpred, zg{i}] = solve_rti(x0, xref, nb, cfg.dstar{i}, zg{i}, P_qp{i}, cfg);
        newpeers{i} = xpred(:, 1:3);

        % 速度合成：xy=MPC k=1 速度，z=纯P 高度保持
        vs = zeros(3,1);
        vs(1:2) = xpred(2, 4:5)';
        ez = xref(1,3) - pos(3,i);
        vs(3) = min(max(cfg.kp_z * ez, -cfg.max_climb), cfg.max_climb);
        vn = norm(vs(1:2));
        if vn > cfg.max_speed, vs(1:2) = vs(1:2) * cfg.max_speed / vn; end
        if any(~isfinite(vs)), vs = zeros(3,1); end
        vel_sp(:,i) = vs;
    end
    peers = newpeers;

    % 被控对象：一阶速度响应
    vel = vel + (vel_sp - vel) * (dt / tau_v);
    pos = pos + vel * dt;
    pos_log(k,:,:) = pos';
end
fprintf('  完成 %.1fs\n', toc);

%% ── 指标计算 ─────────────────────────────────────────────────────────
ref = zeros(Nsteps, 2, n); lead_xy = zeros(Nsteps, 2);
for k = 1:Nsteps
    [lp,~,~] = leader_state(t_log(k), cfg);
    lead_xy(k,:) = lp(1:2);
    for i = 1:n, ref(k,:,i) = lp(1:2) + cfg.offsets(i,1:2); end
end

track_err = squeeze(sqrt(sum((pos_log(:,1:2,:) - ref).^2, 2)));
if n == 1, track_err = track_err(:); end

pairs = []; ds = [];
for i = 1:n
    for m = 1:numel(cfg.nbrs{i})
        j = cfg.nbrs{i}(m);
        if j > i, pairs(end+1,:) = [i j]; ds(end+1) = cfg.dstar{i}(m); end %#ok<AGROW>
    end
end
form_err = zeros(Nsteps,1); min_sp = inf(Nsteps,1);
for k = 1:Nsteps
    errs = [];
    for q = 1:size(pairs,1)
        d = norm(pos_log(k,1:2,pairs(q,1)) - pos_log(k,1:2,pairs(q,2)));
        errs(end+1) = abs(d - ds(q)); %#ok<AGROW>
        min_sp(k) = min(min_sp(k), d);
    end
    if ~isempty(errs), form_err(k) = mean(errs); end
end

alt = squeeze(pos_log(:,3,:)); if n==1, alt=alt(:); end
steady  = t_log > cfg.t_start + 8;
m_form  = mean(form_err(steady));
m_track = mean(max(track_err(steady,:), [], 2));
m_alt   = mean(max(abs(alt(steady,:) - cfg.target_alt), [], 2));
m_minsp = min(min_sp(t_log > 8));

%% ── 三级判定 ─────────────────────────────────────────────────────────
core_ok = m_alt < 0.3 && (n==1 || (m_form < 0.3 && m_minsp > cfg.d_safe));
if ~core_ok
    verdict = 'FAIL';
elseif m_track < 0.5
    verdict = 'PASS';
elseif m_track < 1.5
    verdict = 'REVIEW';
else
    verdict = 'FAIL';
end

fprintf('  form_err=%.3fm  track_err=%.3fm  alt_err=%.3fm  min_sp=%.3fm\n', ...
    m_form, m_track, m_alt, m_minsp);
fprintf('  VERDICT: %s\n', verdict);

res = struct('formation','solo1','mode','circle','form_err',m_form, ...
    'track_err',m_track,'alt_err',m_alt,'min_sp',m_minsp,'verdict',verdict);

%% ── 图 ───────────────────────────────────────────────────────────────
figure('Position',[50 50 1280 720]);
cols = lines(n);

subplot(2,2,1); hold on; grid on; axis equal
for i = 1:n
    plot(pos_log(:,2,i), pos_log(:,1,i), '-', 'Color',cols(i,:),'LineWidth',1.2);
    plot(pos_log(end,2,i), pos_log(end,1,i), 'o', 'Color',cols(i,:),'MarkerFaceColor',cols(i,:),'MarkerSize',7);
end
plot(lead_xy(:,2), lead_xy(:,1), 'k--','LineWidth',1);
xlabel('East [m]'); ylabel('North [m]');
title(sprintf('MPC solo1 circle — 俯视轨迹'));
legend(arrayfun(@(i)sprintf('drone%d',i-1),1:n,'UniformOutput',false),'leader','Location','best');

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
sgtitle(sprintf('solo1 × circle  →  %s  (form=%.3fm  track=%.3fm  minsp=%.2fm)', ...
    verdict, m_form, m_track, m_minsp), 'FontWeight','bold');
end

% ════════════════════════════════════════════════════════════════════════
% 局部函数：完整 MPC 算法，对照 mpc_node.py 注释已标行号
% ════════════════════════════════════════════════════════════════════════

function [p, v, a] = leader_state(t, cfg)
%LEADER_STATE leader 虚拟点位置/速度/加速度（NED）
% 对照 leader_node：t_start 前悬停；line 带端点减速；circle 含向心加速度
alt = cfg.target_alt;
p = [0 0 alt]; v = [0 0 0]; a = [0 0 0];
tau = t - cfg.t_start;
if tau <= 0, return; end

switch cfg.mode
    case 'hover'
        % 原地不动

    case 'line'
        vmax = cfg.lead_v; d = cfg.lead_d; ad = cfg.lead_dec;
        s_dec = d - vmax^2 / (2*ad);
        t1    = s_dec / vmax;
        if tau <= t1
            s = vmax*tau; vx = vmax; ax = 0;
        else
            td = tau - t1;
            if td < vmax/ad
                s = s_dec + vmax*td - 0.5*ad*td^2;
                vx = vmax - ad*td; ax = -ad;
            else
                s = d; vx = 0; ax = 0;
            end
        end
        p(1) = s; v(1) = vx; a(1) = ax;

    case 'circle'   % 圆心在 (-R,0)，t=0 从原点出发，切向 +y 进入
        R = cfg.lead_R; vc = cfg.lead_v; om = vc / R;
        p(1) =  R*cos(om*tau) - R;
        p(2) =  R*sin(om*tau);
        v(1) = -vc*sin(om*tau);
        v(2) =  vc*cos(om*tau);
        a(1) = -vc*om*cos(om*tau);
        a(2) = -vc*om*sin(om*tau);
end
end

% ────────────────────────────────────────────────────────────────────────

function P = precompute(cfg, M)
%PRECOMPUTE 单机 MPC 常量矩阵（M=邻居数）对照 mpc_node._setup_ocp L107-181
N = cfg.N; dt = cfg.mpc_dt; nx = 6; nu = 3;
P.N = N; P.nx = nx; P.nu = nu; P.M = M;
I3 = eye(3);
A  = [I3 dt*I3; zeros(3) I3];
B  = [0.5*dt^2*I3; dt*I3];
P.A = A; P.B = B;
P.nz  = nx*(N+1); P.nuu = nu*N;
P.xidx = @(k) k*nx + (1:nx);
P.uidx = @(k) k*nu + (1:nu);

Phi = zeros(P.nz, nx); Gam = zeros(P.nz, P.nuu); Ak = eye(nx);
Phi(1:nx,:) = Ak;
for k = 1:N
    Ak = A*Ak; Phi(P.xidx(k),:) = Ak;
    for j = 0:k-1
        Gam(P.xidx(k), P.uidx(j)) = A^(k-1-j)*B;
    end
end
P.Phi = Phi; P.Gam = Gam;

P.Q  = diag([cfg.q_pos*[1 1 1], cfg.q_vel*[1 1 1]]);
P.Qe = diag([cfg.q_pos*cfg.q_term_s*[1 1 1], cfg.q_vel*[1 1 1]]);
P.R  = cfg.r_acc * eye(3);
Hx = zeros(P.nz, 1);
for k = 0:N-1, Hx(P.xidx(k)) = 2*diag(P.Q); end
Hx(P.xidx(N)) = 2*diag(P.Qe);
P.Hx_diag = Hx;
P.Hu = 2*kron(eye(N), P.R) + 2*cfg.lm*eye(P.nuu);

vel_rows = []; lv = []; uv = [];
for k = 1:N-1
    ix = P.xidx(k); vel_rows = [vel_rows, ix(4:6)]; %#ok<AGROW>
    lv = [lv; -cfg.max_speed; -cfg.max_speed; -cfg.max_climb]; %#ok<AGROW>
    uv = [uv;  cfg.max_speed;  cfg.max_speed;  cfg.max_climb]; %#ok<AGROW>
end
P.vel_rows = vel_rows;
P.Acon = [eye(P.nuu); Gam(vel_rows,:)];
P.lu = -cfg.max_accel * ones(P.nuu,1);
P.uu =  cfg.max_accel * ones(P.nuu,1);
P.lv = lv; P.uv = uv;
end

% ────────────────────────────────────────────────────────────────────────

function [u0, xpred, ws] = solve_rti(x0, xref, nb, dstar, ws, P, cfg)
%SOLVE_RTI 单次 Gauss-Newton RTI（= acados SQP_RTI 语义）
% 跟踪/输入项精确线性化；碰撞/编队残差在上一拍 xbar 处 GN 线性化
N = P.N; nx = P.nx; M = P.M;
if isempty(ws)
    ws = struct('u', zeros(P.nuu,1), 'y', [], 'xbar', repmat(x0', N+1, 1));
end

Hx = spdiags(P.Hx_diag, 0, P.nz, P.nz);
fx = zeros(P.nz, 1);
for k = 0:N-1, fx(P.xidx(k)) = -2 * P.Q * xref(k+1,:)'; end
fx(P.xidx(N)) = -2 * P.Qe * xref(N+1,:)';

% 碰撞/编队 GN 项（对照 mpc_node L130-135）
swc = sqrt(cfg.w_coll); swf = sqrt(cfg.w_form);
for k = 0:N-1
    ix = P.xidx(k); ixy = ix(1:2);
    xy = ws.xbar(k+1, 1:2)';
    for m = 1:M
        nxy = squeeze(nb(m, k+1, 1:2));
        df = xy - nxy; d = sqrt(df'*df + 1e-6);
        g = swf*df/d; r = swf*(d - dstar(m));         % 编队
        Hx(ixy,ixy) = Hx(ixy,ixy) + 2*(g*g');
        fx(ixy) = fx(ixy) + 2*(r - g'*xy)*g;
        if d < cfg.d_safe                               % 碰撞（激活区）
            g = -swc*df/d; r = swc*(cfg.d_safe - d);
            Hx(ixy,ixy) = Hx(ixy,ixy) + 2*(g*g');
            fx(ixy) = fx(ixy) + 2*(r - g'*xy)*g;
        end
    end
end

phix = P.Phi * x0;
Pu   = (P.Gam'*(Hx*P.Gam) + P.Hu); Pu = (Pu+Pu')/2;
qu   = P.Gam' * (fx + Hx*phix);
l    = [P.lu; P.lv - phix(P.vel_rows)];
u    = [P.uu; P.uv - phix(P.vel_rows)];

[usol, ydual] = admm(Pu, qu, P.Acon, l, u, ws.u, ws.y);
if any(~isfinite(usol)), usol = ws.u; end
xstack = phix + P.Gam*usol;
xpred  = reshape(xstack, nx, N+1)';
u0 = usol(1:P.nu);
ws.u = usol; ws.y = ydual; ws.xbar = xpred;
end

% ────────────────────────────────────────────────────────────────────────

function [u, y] = admm(P, q, A, l, ub, u0, y0)
%ADMM OSQP 风格 ADMM 求解  min ½uᵀPu + qᵀu  s.t. l ≤ Au ≤ ub（无工具箱）
rho = 10.0; sigma = 1e-6; iters = 200; tol = 1e-4;
nu = numel(q);
if isempty(u0), u0 = zeros(nu,1); end
if isempty(y0), y0 = zeros(size(A,1),1); end
u = u0; y = y0; z = min(max(A*u, l), ub);
K = chol(P + sigma*eye(nu) + rho*(A'*A), 'lower');
for it = 1:iters
    u     = K' \ (K \ (sigma*u - q + A'*(rho*z - y)));
    Au    = A*u; z_new = min(max(Au + y/rho, l), ub);
    y     = y + rho*(Au - z_new);
    if norm(Au-z_new,inf)<tol && rho*norm(z_new-z,inf)<tol, break; end
    z = z_new;
end
end
