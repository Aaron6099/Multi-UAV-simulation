function res = animate_mpc(formation, mode)
%ANIMATE_MPC 单场景 MPC 闭环验证 + 屏幕实时 3D 编队动画。
%   res = animate_mpc('trio3','line')
% 与 scenario_run 同建模/指标/verdict（结果一致、可补齐 results.csv），
% 区别：仿真后在可见窗口里逐帧播放编队动画，并另存 MP4 到 ../figures/。
% 真机部署映射：每个 run_mpc_<formation>_<mode>.m 对应一个真机检验单元。
here = fileparts(mfilename('fullpath'));
addpath(here);
evalin('base', sprintf('run(''%s'');', fullfile(fileparts(here), 'init.m')));

cfg = formation_cfg(formation, mode);
if strcmp(mode, 'circle')
    cfg.T = max(cfg.T, 120);   % 动画延长显示稳态圆周，CSV 数据由 scenario_run 独立保存
end
global MPC_CFG %#ok<GVMIS>
MPC_CFG = cfg;
clear mpc_swarm_step                            % 复位 persistent

mdl = build_swarm_model(cfg.n, false);
if ~bdIsLoaded(mdl), load_system(fullfile(here, [mdl '.slx'])); end
for i = 1:cfg.n
    set_param(sprintf('%s/drone_%d/6DOF (Euler Angles)', mdl, i-1), ...
        'xme_0', mat2str(cfg.births(i, :)));
end
set_param(mdl, 'StopTime', num2str(cfg.T));

fprintf('=== MPC %s %s (%d 机, T=%ds) 仿真中...\n', formation, mode, cfg.n, cfg.T);
tic; out = sim(mdl); fprintf('  仿真完成 %.1fs\n', toc);
bdclose(mdl);

% ── 取数 ────────────────────────────────────────────────────────────────
t = out.(sprintf('Xe_%d', 0)).Time;
L = numel(t); n = cfg.n;
pos = zeros(L, 3, n);
for i = 1:n, pos(:, :, i) = out.(sprintf('Xe_%d', i-1)).Data; end

ref = zeros(L, 2, n); lead = zeros(L, 2);
for kk = 1:L
    lp = leader_state(t(kk), cfg);
    lead(kk, :) = lp(1:2);
    for i = 1:n, ref(kk, :, i) = lp(1:2) + cfg.offsets(i, 1:2); end
end

% ── 指标（同 scenario_run 口径）─────────────────────────────────────────
pairs = []; ds = [];
for i = 1:n
    for m = 1:numel(cfg.nbrs{i})
        j = cfg.nbrs{i}(m);
        if j > i, pairs(end+1,:) = [i j]; ds(end+1) = cfg.dstar{i}(m); end %#ok<AGROW>
    end
end
form_err = zeros(L,1); min_sp = inf(L,1);
for kk = 1:L
    errs = [];
    for q = 1:size(pairs,1)
        d = norm(pos(kk,1:2,pairs(q,1)) - pos(kk,1:2,pairs(q,2)));
        errs(end+1) = abs(d - ds(q)); %#ok<AGROW>
        min_sp(kk) = min(min_sp(kk), d);
    end
    if ~isempty(errs), form_err(kk) = mean(errs); end
end
track_err = squeeze(vecnorm(pos(:,1:2,:) - ref, 2, 2));
alt = squeeze(pos(:,3,:)); if n==1, alt = alt(:); track_err = track_err(:); end
steady = t > cfg.t_start + 8;
m_form  = mean(form_err(steady));
m_track = mean(max(track_err(steady,:), [], 2));
m_alt   = mean(max(abs(alt(steady,:) - cfg.target_alt), [], 2));
m_minsp = min(min_sp(t > 8));
core_ok = m_alt < 0.3 && (n==1 || (m_form < 0.3 && m_minsp > cfg.d_safe));
if ~core_ok,            verdict = 'FAIL';
elseif m_track < 0.5,  verdict = 'PASS';
elseif m_track < 1.5,  verdict = 'REVIEW';
else,                  verdict = 'FAIL'; end
res = struct('formation', formation, 'mode', mode, 'form_err', m_form, ...
    'track_err', m_track, 'alt_err', m_alt, 'min_sp', m_minsp, 'verdict', verdict);
fprintf(['  steady: form_err=%.3fm  track_err=%.3fm  alt_err=%.3fm  ' ...
    'min_sp=%.3fm\n  VERDICT: %s\n'], m_form, m_track, m_alt, m_minsp, verdict);

% ── 静态 PNG（与 scenario_run 同图，供报告）────────────────────────────
figdir = fullfile(fileparts(fileparts(here)), 'figures');
fig = figure('Visible','off','Position',[50 50 1280 720]);
subplot(2,2,1); hold on; grid on; axis equal; cols = lines(n);
if strcmp(mode, 'circle')
    th_ref = linspace(0, 2*pi, 300); R_c = cfg.lead_R;
    for i = 1:n
        cx_e = cfg.offsets(i,2); cy_n = cfg.offsets(i,1) - R_c;
        plot(cx_e + R_c*sin(th_ref), cy_n + R_c*cos(th_ref), '--', ...
             'Color', [0.75 0.75 0.75], 'LineWidth', 1.0);
    end
end
for i = 1:n
    plot(pos(:,2,i), pos(:,1,i), '-', 'Color', cols(i,:));
    plot(pos(end,2,i), pos(end,1,i), 'o', 'Color', cols(i,:), 'MarkerFaceColor', cols(i,:));
end
plot(lead(:,2), lead(:,1), 'k--'); xlabel('East [m]'); ylabel('North [m]');
title(sprintf('MPC %s %s — 俯视轨迹', formation, mode));
subplot(2,2,2); plot(t, form_err); grid on; xlabel('t [s]'); ylabel('form\_err [m]'); title('编队误差');
subplot(2,2,3); plot(t, alt); yline(cfg.target_alt,'k--'); grid on; xlabel('t [s]'); ylabel('z NED [m]'); title('高度');
subplot(2,2,4); hold on; grid on;
if ~isempty(pairs), plot(t, min_sp); yline(cfg.d_safe,'r--','d\_safe'); ylabel('min spacing [m]'); title('最小间距');
else, plot(t, track_err); ylabel('track\_err [m]'); title('跟踪误差'); end
xlabel('t [s]');
png = fullfile(figdir, sprintf('mpc_%s_%s.png', formation, mode));
exportgraphics(fig, png, 'Resolution', 120); close(fig);
fprintf('  图: %s\n', png);

% ── 追加 results.csv（去重：同 formation+mode 已存在则跳过）──────────────
csv = fullfile(here, 'results.csv');
exists = false;
if exist(csv, 'file')
    T0 = readtable(csv, 'TextType','string');
    exists = any(T0.formation == string(formation) & T0.mode == string(mode));
end
if exists
    fprintf('  results.csv 已有 %s %s，跳过追加。\n', formation, mode);
else
    fid = fopen(csv, 'a');
    fprintf(fid, '%s,%s,%.4f,%.4f,%.4f,%.4f,%s\n', formation, mode, ...
        m_form, m_track, m_alt, m_minsp, verdict);
    fclose(fid); fprintf('  已追加 results.csv\n');
end

% ── 屏幕实时 3D 动画（并存 MP4）─────────────────────────────────────────
armR = 0.8; FPS = 20; SPEED = 2.0;
dt = t(2) - t(1); stp = max(1, round(SPEED/FPS/dt));
TRAIL = round(15/dt);
if strcmp(mode, 'circle')
    takeoff_end = max(1, round(cfg.t_start / dt));      % 起飞段结束（t_start 前）
    start_k     = max(1, round((cfg.t_start + 50) / dt)); % 稳态圆周开始
    frames = [1:stp:takeoff_end, start_k:stp:L];       % 起飞 + 稳态圆周，跳过收敛螺旋
else
    start_k = 1;
    frames = 1:stp:L;
end
cc = lines(n);
fig = figure('Name', sprintf('MPC %s %s 动画', formation, mode), ...
    'Position', [80 80 1100 720], 'Color', 'w');
ax = axes(fig); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
xlabel(ax,'North [m]'); ylabel(ax,'East [m]'); zlabel(ax,'Alt [m]');
allN = pos(:,1,:); allE = pos(:,2,:);
view(ax, -35, 28); axis(ax,'equal');
xlim(ax,[min(allN(:))-2 max(allN(:))+2]); ylim(ax,[min(allE(:))-2 max(allE(:))+2]); zlim(ax,[0 7]);
plot3(ax, lead(:,1), lead(:,2), -cfg.target_alt*ones(L,1), 'k--', 'LineWidth', 0.7);
if strcmp(mode, 'circle')
    th_ref = linspace(0, 2*pi, 300); R_c = cfg.lead_R;
    for i = 1:n
        cn = cfg.offsets(i,1) - R_c; ce = cfg.offsets(i,2);
        plot3(ax, cn + R_c*cos(th_ref), ce + R_c*sin(th_ref), ...
              -cfg.target_alt*ones(1,300), '--', 'Color', [0.75 0.75 0.75], 'LineWidth', 1.0);
    end
end
hArm1 = gobjects(n,1); hArm2 = gobjects(n,1); hDot = gobjects(n,1); hTrail = gobjects(n,1);
for d = 1:n
    hTrail(d) = plot3(ax,nan,nan,nan,'-','Color',[cc(d,:) 0.45],'LineWidth',1.0);
    hArm1(d)  = plot3(ax,nan,nan,nan,'-','Color',cc(d,:),'LineWidth',2.5);
    hArm2(d)  = plot3(ax,nan,nan,nan,'-','Color',cc(d,:),'LineWidth',2.5);
    hDot(d)   = plot3(ax,nan,nan,nan,'o','Color',cc(d,:),'MarkerFaceColor',cc(d,:),'MarkerSize',6);
end
hForm = plot3(ax,nan,nan,nan,':','Color',[0.4 0.4 0.4],'LineWidth',1.0);
hTitle = title(ax,'');
legend(ax, hDot, arrayfun(@(d) sprintf('d%d',d-1),1:n,'UniformOutput',false), 'Location','northeast');

mp4 = fullfile(figdir, sprintf('mpc_%s_%s_anim.mp4', formation, mode));
vw = VideoWriter(mp4, 'MPEG-4'); vw.FrameRate = FPS; open(vw);
fprintf('  播放动画 (%d 帧) → 另存 %s\n', numel(frames), mp4);
for k = frames
    for d = 1:n
        p = pos(k,:,d); a = -p(3);
        set(hArm1(d),'XData',p(1)+[-armR armR],'YData',p(2)+[0 0],'ZData',[a a]);
        set(hArm2(d),'XData',p(1)+[0 0],'YData',p(2)+[-armR armR],'ZData',[a a]);
        set(hDot(d),'XData',p(1),'YData',p(2),'ZData',a);
        i0 = max(1 + (k>=start_k)*(start_k-1), k-TRAIL); % 起飞段从1，稳态段从start_k
        set(hTrail(d),'XData',pos(i0:k,1,d),'YData',pos(i0:k,2,d),'ZData',-pos(i0:k,3,d));
    end
    poly = zeros(n+1,3);
    for d = 1:n, poly(d,:) = pos(k,:,d); end
    poly(n+1,:) = pos(k,:,1);
    set(hForm,'XData',poly(:,1),'YData',poly(:,2),'ZData',-poly(:,3));
    set(hTitle,'String',sprintf('MPC %s %s — t=%.1fs (2x)  [%s]', formation, mode, t(k), verdict));
    drawnow; writeVideo(vw, getframe(fig));
end
close(vw);
fprintf('  动画完成: %s\n', mp4);
end
