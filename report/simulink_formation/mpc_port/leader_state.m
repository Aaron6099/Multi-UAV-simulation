function [p, v, a] = leader_state(t, cfg)
%LEADER_STATE leader 虚拟点位置/速度/加速度（NED，z 恒为 target_alt）
% 与 leader_node 同语义：t_start 前悬停在原点上方；line 带端点减速；
% circle 广播向心加速度（mpc_node 参考轨迹二阶预测依赖它）。
alt = cfg.target_alt;
p = [0 0 alt]; v = [0 0 0]; a = [0 0 0];
tau = t - cfg.t_start;
if tau <= 0, return; end

switch cfg.mode
    case 'hover'
        % 原地

    case 'line'   % 北向(+x)，恒速 → 端点减速停在 d
        vmax = cfg.lead_v; d = cfg.lead_d; ad = cfg.lead_dec;
        s_dec = d - vmax^2 / (2*ad);          % 开始减速的里程
        t1 = s_dec / vmax;                    % 减速开始时刻(相对 tau)
        if tau <= t1
            s = vmax * tau;            vx = vmax;            ax = 0;
        else
            td = tau - t1;
            if td < vmax/ad
                s  = s_dec + vmax*td - 0.5*ad*td^2;
                vx = vmax - ad*td;     ax = -ad;
            else
                s = d; vx = 0; ax = 0;
            end
        end
        p(1) = s; v(1) = vx; a(1) = ax;

    case 'circle' % 起点原点，切向 +y 进入，圆心 (-R,0)
        R = cfg.lead_R; vc = cfg.lead_v; om = vc / R;
        p(1) = R*cos(om*tau) - R;
        p(2) = R*sin(om*tau);
        v(1) = -vc*sin(om*tau);
        v(2) =  vc*cos(om*tau);
        a(1) = -vc*om*cos(om*tau);    % 向心加速度 = -ω²(p-c)
        a(2) = -vc*om*sin(om*tau);
end
end
