function res = scenario_run(formation, mode)
%SCENARIO_RUN 单场景闭环验证：建模 → 仿真 → 指标 → verdict → 图
%   res = scenario_run('pair2','line')
% 真机部署映射：每个 run_mpc_<formation>_<mode>.m 入口对应一个真机检验单元。
here = fileparts(mfilename('fullpath'));
addpath(here);
% 被控对象物理参数必须进 base 工作区（模型变量解析）
evalin('base', sprintf('run(''%s'');', fullfile(fileparts(here), 'init.m')));

cfg = formation_cfg(formation, mode);
global MPC_CFG
MPC_CFG = cfg;
clear mpc_swarm_step                            % 复位 persistent（多场景连跑）

mdl = build_swarm_model(cfg.n, false);
if ~bdIsLoaded(mdl), load_system(fullfile(here, [mdl '.slx'])); end

% 出生位置注入（地面 z=0，与 SITL 一致；MPC 自行爬升到 target_alt）
for i = 1:cfg.n
    set_param(sprintf('%s/drone_%d/6DOF (Euler Angles)', mdl, i-1), ...
        'xme_0', mat2str(cfg.births(i, :)));
end
set_param(mdl, 'StopTime', num2str(cfg.T));

fprintf('=== MPC %s %s (%d 机, T=%ds) 仿真中...\n', formation, mode, cfg.n, cfg.T);
tic; out = sim(mdl); fprintf('  仿真完成 %.1fs\n', toc);
bdclose(mdl);   % xme_0 已改 → 丢弃改动关闭，下个场景重新 load+注入

% ── 取数 ────────────────────────────────────────────────────────────────
t = out.(sprintf('Xe_%d', 0)).Time;
L = numel(t); n = cfg.n;
pos = zeros(L, 3, n);
for i = 1:n
    pos(:, :, i) = out.(sprintf('Xe_%d', i-1)).Data;
end

% 参考(各机) + leader
ref = zeros(L, 2, n); lead = zeros(L, 2);
for kk = 1:L
    [lp, ~, ~] = leader_state(t(kk), cfg);
    lead(kk, :) = lp(1:2);
    for i = 1:n
        ref(kk, :, i) = lp(1:2) + cfg.offsets(i, 1:2);
    end
end

% ── 指标 ────────────────────────────────────────────────────────────────
% 编队误差: 邻居对 |d_ij - d*|平均；solo1 用跟踪误差替代
pairs = []; ds = [];
for i = 1:n
    for m = 1:numel(cfg.nbrs{i})
        j = cfg.nbrs{i}(m);
        if j > i, pairs(end+1, :) = [i j]; ds(end+1) = cfg.dstar{i}(m); end %#ok<AGROW>
    end
end
form_err = zeros(L, 1); min_sp = inf(L, 1);
for kk = 1:L
    errs = [];
    for q = 1:size(pairs, 1)
        d = norm(pos(kk,1:2,pairs(q,1)) - pos(kk,1:2,pairs(q,2)));
        errs(end+1) = abs(d - ds(q)); %#ok<AGROW>
        min_sp(kk) = min(min_sp(kk), d);
    end
    if ~isempty(errs), form_err(kk) = mean(errs); end
end
track_err = squeeze(vecnorm(pos(:,1:2,:) - ref, 2, 2));   % L x n
alt = squeeze(pos(:, 3, :)); if n == 1, alt = alt(:); track_err = track_err(:); end

steady = t > cfg.t_start + 8;                  % 稳态窗口
m_form  = mean(form_err(steady));
m_track = mean(max(track_err(steady, :), [], 2));
m_alt   = mean(max(abs(alt(steady, :) - cfg.target_alt), [], 2));
m_minsp = min(min_sp(t > 8));                  % 起飞后全程

% 三级判定（与 ROS verify_formation.py 同口径）：队形+安全 硬指标；
% 共模跟踪滞后(队形完好、纯速度内环滞后) = REVIEW 非缺陷
core_ok = m_alt < 0.3 && (n == 1 || (m_form < 0.3 && m_minsp > cfg.d_safe));
if ~core_ok
    verdict = 'FAIL';
elseif m_track < 0.5
    verdict = 'PASS';
elseif m_track < 1.5
    verdict = 'REVIEW';   % 共模滞后：队形/间距/高度全好，仅整体落后参考
else
    verdict = 'FAIL';
end
res = struct('formation', formation, 'mode', mode, 'form_err', m_form, ...
    'track_err', m_track, 'alt_err', m_alt, 'min_sp', m_minsp, 'verdict', verdict);

fprintf(['  steady: form_err=%.3fm  track_err=%.3fm  alt_err=%.3fm  ' ...
    'min_sp=%.3fm\n  VERDICT: %s\n'], m_form, m_track, m_alt, m_minsp, verdict);

% ── 图 ──────────────────────────────────────────────────────────────────
try
    fig = figure('Visible', 'off', 'Position', [50 50 1280 720]);
    subplot(2,2,1); hold on; grid on; axis equal
    cols = lines(n);
    % 圆周模式：先画灰色参考圆（每机），严谨对照 SITL 轨迹
    if strcmp(mode, 'circle')
        th = linspace(0, 2*pi, 300);
        R_c = cfg.lead_R;
        for i = 1:n
            cx_e = cfg.offsets(i,2);
            cy_n = cfg.offsets(i,1) - R_c;
            plot(cx_e + R_c*sin(th), cy_n + R_c*cos(th), '--', ...
                 'Color', [0.75 0.75 0.75], 'LineWidth', 1.0);
        end
    end
    for i = 1:n
        plot(pos(:,2,i), pos(:,1,i), '-', 'Color', cols(i,:));
        plot(pos(end,2,i), pos(end,1,i), 'o', 'Color', cols(i,:), 'MarkerFaceColor', cols(i,:));
    end
    plot(lead(:,2), lead(:,1), 'k--');
    xlabel('East [m]'); ylabel('North [m]');
    title(sprintf('MPC %s %s — 俯视轨迹', formation, mode));
    subplot(2,2,2); plot(t, form_err); grid on
    xlabel('t [s]'); ylabel('form\_err [m]'); title('编队误差(邻居对均值)');
    subplot(2,2,3); plot(t, alt); yline(cfg.target_alt, 'k--'); grid on
    xlabel('t [s]'); ylabel('z NED [m]'); title('高度');
    subplot(2,2,4); hold on; grid on
    if ~isempty(pairs)
        plot(t, min_sp); yline(cfg.d_safe, 'r--', 'd\_safe');
        ylabel('min spacing [m]'); title('最小间距');
    else
        plot(t, track_err); ylabel('track\_err [m]'); title('跟踪误差');
    end
    xlabel('t [s]');
    fig_dir = fullfile(fileparts(fileparts(here)), 'figures');
    if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end
    png = fullfile(fig_dir, sprintf('mpc_%s_%s.png', formation, mode));
    exportgraphics(fig, png, 'Resolution', 120);
    close(fig);
    fprintf('  图: %s\n', png);
catch ME
    fprintf('  [警告] 图生成失败: %s\n', ME.message);
    try, close(fig); catch, end
end

% 结果累积 CSV
csv = fullfile(here, 'results.csv');
if ~exist(csv, 'file')
    fid = fopen(csv, 'w');
    fprintf(fid, 'formation,mode,form_err_m,track_err_m,alt_err_m,min_sp_m,verdict\n');
else
    fid = fopen(csv, 'a');
end
fprintf(fid, '%s,%s,%.4f,%.4f,%.4f,%.4f,%s\n', formation, mode, ...
    m_form, m_track, m_alt, m_minsp, verdict);
fclose(fid);
end
