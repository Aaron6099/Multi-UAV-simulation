function [u, y] = qp_admm(P, q, A, l, ub, u0, y0)
%QP_ADMM OSQP 风格 ADMM 求解  min ½uᵀPu + qᵀu  s.t. l ≤ Au ≤ ub
% 小规模稠密专用（本仓 MPC：90 变量 / ~180 约束）。无工具箱依赖。
%   u0,y0 热启动（可 []）。
rho = 10.0; sigma = 1e-6; iters = 200; tol = 1e-4;

nu = numel(q);
if isempty(u0), u0 = zeros(nu, 1); end
if isempty(y0), y0 = zeros(size(A,1), 1); end
u = u0; y = y0;
z = min(max(A*u, l), ub);

K = chol(P + sigma*eye(nu) + rho*(A'*A), 'lower');
for it = 1:iters
    rhs = sigma*u - q + A'*(rho*z - y);
    u = K' \ (K \ rhs);
    Au = A*u;
    z_new = min(max(Au + y/rho, l), ub);
    y = y + rho*(Au - z_new);
    r_prim = norm(Au - z_new, inf);
    r_dual = rho * norm(z_new - z, inf);
    z = z_new;
    if r_prim < tol && r_dual < tol, break; end
end
end
