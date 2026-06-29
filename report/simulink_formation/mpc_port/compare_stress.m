function T = compare_stress()
%COMPARE_STRESS 约束压力对照：MPC vs 分布式二阶一致性 在 S11 扰动出生下的安全裕度/精度。
%   场景=cross5 + S11 birth_override（对照 scenarios.yaml / SITL S11），僚机从打乱位置
%   收敛到标称队形槽并随 leader 机动。起点(births)与目标(offsets)解耦：仅 births 打乱。
%   关键发现：理想 Simulink 动力学下二阶一致性(尤其带前馈)并不灾难性失败、亦不撞机；
%   但其安全裕度极薄(min_sp 仅略高于 d_safe)、收敛精度差。MPC 凭内生约束(w_coll/限幅)
%   保持显著更大的间距裕度与更高精度——这正是真实 SITL(EKF/通信/时序退化吃掉薄裕度→
%   S11 曾 min_sp 0.11m)需为一致性外挂 safety_filter(a657c57)、而 MPC 无需的原因。
%   指标：worst min_sp、裕度(=worst-d_safe)、违规帧、末态队形精度。canonical .slx 不改。
global MPC_CFG CONS_CFG

here    = fileparts(mfilename('fullpath'));
fig_dir = fullfile(fileparts(fileparts(here)), 'figures');
addpath(here);
evalin('base', sprintf('run(''%s'');', fullfile(fileparts(here), 'init.m')));

% 运行清单：{mode, birth('nominal'|'s11'), ctrl('mpc'|'cons'), ff}
specs = {
    {'hover','nominal','mpc',  []  }, {'hover','nominal','cons', true }, ...
    {'hover','s11',    'mpc',  []  }, {'hover','s11',    'cons', true }, {'hover','s11','cons', false}, ...
    {'line', 'nominal','mpc',  []  }, {'line', 'nominal','cons', true }, ...
    {'line', 's11',    'mpc',  []  }, {'line', 's11',    'cons', true }, {'line', 's11', 'cons', false} ...
};

rows = {};
MS = struct();   % min_sp 时序缓存（s11 条件画图用）

for q = 1:numel(specs)
    mode = specs{q}{1}; birth = specs{q}{2}; ctrl = specs{q}{3}; ff = specs{q}{4};
    formation = 'cross5';
    switch ctrl
        case 'mpc'
            cfg = formation_cfg(formation, mode); tagc = 'MPC';
        case 'cons'
            cfg = consensus_cfg(formation, mode, ff);
            tagc = ['CONS' ternary(ff,'+ff','-noff')];
    end
    if strcmp(birth,'s11'), cfg.births = s11_births(); end   % 仅改起点，offsets 仍标称

    switch ctrl
        case 'mpc'
            MPC_CFG = cfg; clear mpc_swarm_step
            mdl = sprintf('mpc_swarm_%d', cfg.n);
            if bdIsLoaded(mdl), bdclose(mdl); end
            build_swarm_model(cfg.n, false);
        case 'cons'
            CONS_CFG = cfg; clear consensus_swarm_step
            mdl = sprintf('consensus_swarm_%d', cfg.n);
            if bdIsLoaded(mdl), bdclose(mdl); end
            build_consensus_model(cfg.n, false);
    end
    if ~bdIsLoaded(mdl), load_system(fullfile(here,[mdl '.slx'])); end
    for i = 1:cfg.n
        set_param(sprintf('%s/drone_%d/6DOF (Euler Angles)', mdl, i-1), ...
            'xme_0', mat2str(cfg.births(i,:)));
    end
    set_param(mdl, 'StopTime', num2str(cfg.T));

    fprintf('  [run] %s %s birth=%s (T=%ds)...\n', tagc, mode, birth, cfg.T);
    out = sim(mdl); bdclose(mdl);

    % ── 指标 ──
    t = out.(sprintf('Xe_%d',0)).Time; L = numel(t); n = cfg.n;
    pos = zeros(L,3,n);
    for i = 1:n, pos(:,:,i) = out.(sprintf('Xe_%d',i-1)).Data; end
    min_sp = inf(L,1);                       % 全机对最小间距（全局安全）
    for kk = 1:L
        for a = 1:n, for b = a+1:n
            min_sp(kk) = min(min_sp(kk), norm(pos(kk,1:2,a)-pos(kk,1:2,b)));
        end; end
    end
    ref = zeros(L,2,n);
    for kk = 1:L
        [lp,~,~] = leader_state(t(kk), cfg);
        for i = 1:n, ref(kk,:,i) = lp(1:2)+cfg.offsets(i,1:2); end
    end
    track_err = squeeze(vecnorm(pos(:,1:2,:)-ref,2,2));
    steady = t > cfg.t_start + 8;
    m_track   = mean(max(track_err(steady,:),[],2));   % 稳态跟踪
    final_acc = max(track_err(end,:));                 % 末态队形精度
    valid = t > 0.5;
    worst_sp = min(min_sp(valid));
    margin   = worst_sp - cfg.d_safe;
    n_viol   = sum(min_sp(valid) < cfg.d_safe);

    rows(end+1,:) = {mode, birth, tagc, worst_sp, margin, n_viol, final_acc, m_track}; %#ok<AGROW>
    fprintf('       worst_min_sp=%.3f  裕度=%+.3f  违规帧=%d  末态精度=%.3f  稳态track=%.3f\n', ...
            worst_sp, margin, n_viol, final_acc, m_track);
    if strcmp(birth,'s11')
        MS.(sprintf('%s_%s', mode, tagc2key(tagc))) = struct('t',t,'min_sp',min_sp,'ds',cfg.d_safe);
    end
end

T = cell2table(rows, 'VariableNames', {'mode','births','ctrl', ...
    'worst_min_sp_m','margin_m','viol_frames','final_acc_m','track_err_m'});
writetable(T, fullfile(here, 'compare_stress.csv'));
disp(T);

% ── 图：S11 扰动下 min_sp(t)，hover/line 各一格，MPC vs 一致性(±ff) ──
try
    if ~exist(fig_dir,'dir'), mkdir(fig_dir); end
    fig = figure('Visible','off','Position',[40 40 1180 460]);
    modes = {'hover','line'};
    for s = 1:2
        md = modes{s};
        subplot(1,2,s); hold on; grid on
        plot_if(MS, sprintf('%s_mpc', md),     'MPC');
        plot_if(MS, sprintf('%s_consff', md),  '二阶一致性+ff');
        plot_if(MS, sprintf('%s_consnoff', md),'二阶一致性-noff');
        yline(1.5,'r--','d\_safe','LineWidth',1.2,'HandleVisibility','off');
        xlabel('t [s]'); ylabel('最小间距 [m]');
        title(sprintf('cross5 %s + S11 扰动：最小间距', md));
        legend('Location','best');
    end
    png = fullfile(fig_dir, 'compare_stress.png');
    exportgraphics(fig, png, 'Resolution', 120); close(fig);
    fprintf('\nCSV → compare_stress.csv\n图 → %s\n', png);
catch ME
    fprintf('[警告] 图生成失败: %s\n', ME.message);
end
end

function s11 = s11_births()
% cross5 S11 birth_override（对照 config/scenarios.yaml）；标称槽 [0,0;0,3;0,-3;3,0;-3,0]
s11 = [1.0 0.8 0; 0.5 3.4 0; -0.7 -2.6 0; 3.5 -0.6 0; -2.8 0.9 0];
end

function k = tagc2key(tagc)
k = lower(strrep(strrep(tagc,'+',''),'-',''));   % 'CONS+ff'->'consff', 'MPC'->'mpc'
end

function plot_if(MS, key, lbl)
if isfield(MS, key)
    plot(MS.(key).t, MS.(key).min_sp, 'LineWidth', 1.8, 'DisplayName', lbl);
end
end

function r = ternary(c,a,b), if c, r=a; else, r=b; end; end
