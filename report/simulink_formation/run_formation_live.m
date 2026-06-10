%% run_formation_live.m — trio3 圆周编队 3D 动画回放（可视化实跑画面）
% 在 MATLAB 中直接运行: 跑 3 机 Simulink 仿真 → 3D 动画回放（实时可看）
% 同时导出 ../figures/simulink_trio3_circle_formation.mp4
%
% 若只想看单机的 Simulink 原生 UAV Animation 窗口:
%   open_system('pid_formation_multi'); 取消注释 UAV Animation 块后正常运行。

clear; clc; close all;
run('init.m');

mdl = 'pid_formation_multi';
if ~bdIsLoaded(mdl), load_system(mdl); end
blk6dof = [mdl '/6DOF (Euler Angles)'];

%% ── 1. trio3 circle 场景（与 verify_formation_simulink 场景3一致）──────────
BIRTH = [ 3     0      0;
         -1.5   2.598  0;
         -1.5  -2.598  0];
T = 65; dtc = 0.05; t = (0:dtc:T)';
R = 10; v = 1.5; om = v / R; tc = max(t - 10, 0);
vx = -v * sin(om * tc) .* (t >= 10);
vy =  v * cos(om * tc) .* (t >= 10);
vz = -1.0 * (t < 5);
vx_ts = timeseries(vx, t); vy_ts = timeseries(vy, t); vz_ts = timeseries(vz, t);

n = 3;
fprintf('仿真 3 机 trio3 circle (各 %ds)...\n', T);
XeAll = cell(n, 1); EulAll = cell(n, 1); tAll = [];
for d = 1:n
    simIn = Simulink.SimulationInput(mdl);
    simIn = simIn.setBlockParameter(blk6dof, 'xme_0', mat2str(BIRTH(d, :)));
    simIn = simIn.setModelParameter('StopTime', num2str(T));
    simIn = simIn.setVariable('vx_ts', vx_ts);
    simIn = simIn.setVariable('vy_ts', vy_ts);
    simIn = simIn.setVariable('vz_ts', vz_ts);
    fprintf('  drone%d ... ', d-1); tic;
    out = sim(simIn);
    XeAll{d} = out.logsout.getElement('Xe').Values.Data;
    try, EulAll{d} = out.logsout.getElement('Euler').Values.Data; catch, EulAll{d} = []; end
    tAll = out.logsout.getElement('Xe').Values.Time;
    fprintf('done (%.1f s)\n', toc);
end
L = min(cellfun(@(x) size(x,1), XeAll));
tAll = tAll(1:L);
for d = 1:n, XeAll{d} = XeAll{d}(1:L, :); end

%% ── 2. 3D 动画回放 + 导出 MP4 ───────────────────────────────────────────
OUTDIR = fullfile(fileparts(pwd), 'figures');
vidfile = fullfile(OUTDIR, 'simulink_trio3_circle_formation.mp4');
FPS = 20; SPEED = 2.0;                       % 2 倍速回放
step = max(1, round(SPEED / FPS / (tAll(2)-tAll(1))));
frames = 1:step:L;

cols = {[0 0.447 0.741], [0.851 0.325 0.098], [0.466 0.674 0.188]};
armR = 0.8;   % 显示用机臂半径(放大便于观看)

fig = figure('Name', 'trio3 circle formation', 'Position', [80 80 1100 750], ...
    'Color', 'w');
ax = axes(fig); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
xlabel('X north [m]'); ylabel('Y east [m]'); zlabel('Alt [m]');
view(ax, -35, 28); axis(ax, 'equal');
xlim(ax, [-26 10]); ylim(ax, [-15 15]); zlim(ax, [0 7]);

% 领队参考圆 (圆心在 [3-10, 0] 西侧, 半径10, 高5)
th = linspace(0, 2*pi, 200);
plot3(ax, (3-R) + R*cos(th), R*sin(th), 5*ones(size(th)), 'k--', 'LineWidth', 0.7);

% 图元: 每机 机体十字 + 中心点 + 尾迹
hArm1 = gobjects(n,1); hArm2 = gobjects(n,1); hDot = gobjects(n,1); hTrail = gobjects(n,1);
for d = 1:n
    hTrail(d) = plot3(ax, nan, nan, nan, '-', 'Color', [cols{d} 0.45], 'LineWidth', 1.0);
    hArm1(d)  = plot3(ax, nan, nan, nan, '-', 'Color', cols{d}, 'LineWidth', 2.5);
    hArm2(d)  = plot3(ax, nan, nan, nan, '-', 'Color', cols{d}, 'LineWidth', 2.5);
    hDot(d)   = plot3(ax, nan, nan, nan, 'o', 'Color', cols{d}, ...
        'MarkerFaceColor', cols{d}, 'MarkerSize', 6);
end
% 机间连线（队形三角）
hForm = plot3(ax, nan, nan, nan, ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0);
hTitle = title(ax, '');
legend(ax, [hDot(1) hDot(2) hDot(3)], {'d0','d1','d2'}, 'Location', 'northeast');

vw = VideoWriter(vidfile, 'MPEG-4'); vw.FrameRate = FPS; open(vw);
fprintf('渲染动画 %d 帧 → %s\n', numel(frames), vidfile);

TRAIL = round(15 / (tAll(2)-tAll(1)));       % 15s 尾迹
for k = frames
    for d = 1:n
        p = XeAll{d}(k, :); alt = -p(3);
        % 十字机臂 (XY 平面)
        set(hArm1(d), 'XData', p(1)+[-armR armR], 'YData', p(2)+[0 0],   'ZData', [alt alt]);
        set(hArm2(d), 'XData', p(1)+[0 0],   'YData', p(2)+[-armR armR], 'ZData', [alt alt]);
        set(hDot(d),  'XData', p(1), 'YData', p(2), 'ZData', alt);
        i0 = max(1, k-TRAIL);
        set(hTrail(d), 'XData', XeAll{d}(i0:k,1), 'YData', XeAll{d}(i0:k,2), ...
            'ZData', -XeAll{d}(i0:k,3));
    end
    % 队形三角连线
    tri = [XeAll{1}(k,:); XeAll{2}(k,:); XeAll{3}(k,:); XeAll{1}(k,:)];
    set(hForm, 'XData', tri(:,1), 'YData', tri(:,2), 'ZData', -tri(:,3));
    set(hTitle, 'String', sprintf('Simulink PID trio3 Circle — t = %.1f s  (2x speed)', tAll(k)));
    drawnow limitrate;
    writeVideo(vw, getframe(fig));
end
close(vw);
fprintf('完成: %s\n', vidfile);
fprintf('(交互运行本脚本即可在 figure 里实时观看; MP4 可直接双击播放)\n');
