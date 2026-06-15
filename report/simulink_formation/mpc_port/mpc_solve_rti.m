function [u0, xpred, ws] = mpc_solve_rti(x0, xref, nb, dstar, ws, P, cfg)
%MPC_SOLVE_RTI 一次 Gauss-Newton SQP 迭代（= acados SQP_RTI 语义），凝聚 + ADMM
%   x0    : 6x1 当前状态(世界 NED)
%   xref  : (N+1)x6 参考轨迹
%   nb    : M x (N+1) x 3 邻居预测轨迹（M=0 → []）
%   dstar : 1xM 期望间距
%   ws    : 热启动 struct(.u .y .xbar)，[] = 冷启动
% 跟踪/输入项线性残差(精确)；碰撞/编队残差在上一拍预测 xbar 处线性化(GN)。
N = P.N; nx = P.nx; M = P.M;

if isempty(ws)
    ws = struct('u', zeros(P.nuu,1), 'y', [], ...
                'xbar', repmat(x0', N+1, 1));     % 冷启动：悬停在 x0
end

% x 堆叠上的 Hessian 对角 + 梯度（跟踪项）
Hx = spdiags(P.Hx_diag, 0, P.nz, P.nz);
fx = zeros(P.nz, 1);
for k = 0:N-1
    fx(P.xidx(k)) = -2 * P.Q * xref(k+1, :)';
end
fx(P.xidx(N)) = -2 * P.Qe * xref(N+1, :)';

% 碰撞/编队 GN 项（仅 x,y 两元素 rank-1）
swc = sqrt(cfg.w_coll); swf = sqrt(cfg.w_form);
for k = 0:N-1
    ix = P.xidx(k); ixy = ix(1:2);
    xy = ws.xbar(k+1, 1:2)';
    for m = 1:M
        nxy  = squeeze(nb(m, k+1, 1:2));
        df = xy - nxy;
        d  = sqrt(df'*df + 1e-6);
        g = swf * df / d;  r = swf * (d - dstar(m));      % 编队
        Hx(ixy, ixy) = Hx(ixy, ixy) + 2*(g*g');
        fx(ixy) = fx(ixy) + 2*(r - g'*xy)*g;
        if d < cfg.d_safe                                  % 碰撞(激活区)
            g = -swc * df / d;  r = swc * (cfg.d_safe - d);
            Hx(ixy, ixy) = Hx(ixy, ixy) + 2*(g*g');
            fx(ixy) = fx(ixy) + 2*(r - g'*xy)*g;
        end
    end
end

% 凝聚到 u 空间: xstack = Phi x0 + Gam u
phix = P.Phi * x0;
HxG  = Hx * P.Gam;
Pu = P.Gam' * HxG + P.Hu;
Pu = (Pu + Pu') / 2;
qu = P.Gam' * (fx + Hx * phix);

l = [P.lu; P.lv - phix(P.vel_rows)];
u = [P.uu; P.uv - phix(P.vel_rows)];

[usol, ydual] = qp_admm(Pu, qu, P.Acon, l, u, ws.u, ws.y);
if any(~isfinite(usol)), usol = ws.u; end

xstack = phix + P.Gam * usol;
xpred = reshape(xstack, nx, N+1)';
u0 = usol(1:P.nu);
ws.u = usol; ws.y = ydual; ws.xbar = xpred;
end
