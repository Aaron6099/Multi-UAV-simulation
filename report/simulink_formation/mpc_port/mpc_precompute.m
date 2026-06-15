function P = mpc_precompute(cfg, M)
%MPC_PRECOMPUTE 单机 MPC 常量结构（M = 该机邻居数）
% 双积分精确离散化 + 凝聚矩阵(x = Phi x0 + Gam u) + 常量约束矩阵。
% QP 用自带 qp_admm（机器无优化工具箱），对照 mpc_node._setup_ocp。
N = cfg.N; dt = cfg.mpc_dt; nx = 6; nu = 3;
P.N = N; P.nx = nx; P.nu = nu; P.M = M;

I3 = eye(3);
A = [I3 dt*I3; zeros(3) I3];
B = [0.5*dt^2*I3; dt*I3];
P.A = A; P.B = B;

P.nz  = nx*(N+1);            % x 堆叠长度
P.nuu = nu*N;                % u 堆叠长度
P.xidx = @(k) k*nx + (1:nx);             % k = 0..N
P.uidx = @(k) k*nu + (1:nu);             % k = 0..N-1 (u 堆叠内)

% 凝聚: xstack = Phi*x0 + Gam*ustack
Phi = zeros(P.nz, nx); Gam = zeros(P.nz, P.nuu);
Ak = eye(nx);
Phi(1:nx, :) = Ak;
for k = 1:N
    Ak = A * Ak;
    Phi(P.xidx(k), :) = Ak;
    for j = 0:k-1
        Gam(P.xidx(k), P.uidx(j)) = A^(k-1-j) * B;
    end
end
P.Phi = Phi; P.Gam = Gam;

% 跟踪代价权重（stage / terminal），cost = ½wᵀHw + fᵀw 约定 → 系数 2
P.Q  = diag([cfg.q_pos*[1 1 1], cfg.q_vel*[1 1 1]]);
P.Qe = diag([cfg.q_pos*cfg.q_term_s*[1 1 1], cfg.q_vel*[1 1 1]]);
P.R  = cfg.r_acc * eye(3);
Hx = zeros(P.nz, 1);
for k = 0:N-1, Hx(P.xidx(k)) = 2*diag(P.Q); end
Hx(P.xidx(N)) = 2*diag(P.Qe);
P.Hx_diag = Hx;                                   % x 堆叠上的常量对角
P.Hu = 2 * kron(eye(N), P.R) + 2*cfg.lm*eye(P.nuu);

% 约束: u 箱 + 中间级速度界(acados idxbx: k=1..N-1)
vel_rows = [];
lv = []; uv = [];
for k = 1:N-1
    ix = P.xidx(k);
    vel_rows = [vel_rows, ix(4:6)]; %#ok<AGROW>
    lv = [lv; -cfg.max_speed; -cfg.max_speed; -cfg.max_climb]; %#ok<AGROW>
    uv = [uv;  cfg.max_speed;  cfg.max_speed;  cfg.max_climb]; %#ok<AGROW>
end
P.vel_rows = vel_rows;
P.Acon = [eye(P.nuu); Gam(vel_rows, :)];          % 常量
P.lu = -cfg.max_accel * ones(P.nuu, 1);
P.uu =  cfg.max_accel * ones(P.nuu, 1);
P.lv = lv; P.uv = uv;
end
