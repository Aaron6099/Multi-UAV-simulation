%% render_all_animations.m — 所有场景 3D 动画 MP4 批量渲染
% pair2 hover / pair2 line / pair2 perturbed /
% trio3 hover / trio3 line / trio3 circle 扰动出生（trio3 circle 标准版已有）
% 输出 ../figures/simulink_<scenario>_formation.mp4

clear; clc; close all;
run('init.m');

mdl = 'pid_formation_multi';
if ~bdIsLoaded(mdl), load_system(mdl); end
blk6dof = [mdl '/6DOF (Euler Angles)'];
OUTDIR = fullfile(fileparts(pwd), 'figures');

BIRTH_PAIR2      = [0 0 0; -3 0 0];
BIRTH_PAIR2_PERT = [0.5 0.3 0; -2.7 -0.4 0];
BIRTH_TRIO3      = [3 0 0; -1.5 2.598 0; -1.5 -2.598 0];
BIRTH_TRIO3_PERT = [3.3 0.5 0; -1.2 2.8 0; -1.8 -2.3 0];
dtc = 0.05;

scn = struct([]);

% pair2 hover, 30s
T = 30; t = (0:dtc:T)';
scn(1).name = 'pair2_hover'; scn(1).births = BIRTH_PAIR2; scn(1).T = T; scn(1).t = t;
scn(1).vx = zeros(size(t)); scn(1).vy = zeros(size(t)); scn(1).vz = -1.0*(t<5);
scn(1).mass = []; scn(1).xl = [-8 5]; scn(1).yl = [-5 5];
scn(1).title = 'Simulink PID pair2 Hover';
scn(1).refcircle = false;

% pair2 line v=0.5 d=20, 60s
T = 60; t = (0:dtc:T)';
scn(2).name = 'pair2_line'; scn(2).births = BIRTH_PAIR2; scn(2).T = T; scn(2).t = t;
scn(2).vx = 0.5*(t>=10 & t<50); scn(2).vy = zeros(size(t)); scn(2).vz = -1.0*(t<5);
scn(2).mass = []; scn(2).xl = [-6 24]; scn(2).yl = [-6 6];
scn(2).title = 'Simulink PID pair2 Line v=0.5 m/s';
scn(2).refcircle = false;

% trio3 hover, 30s
T = 30; t = (0:dtc:T)';
scn(3).name = 'trio3_hover'; scn(3).births = BIRTH_TRIO3; scn(3).T = T; scn(3).t = t;
scn(3).vx = zeros(size(t)); scn(3).vy = zeros(size(t)); scn(3).vz = -1.0*(t<5);
scn(3).mass = []; scn(3).xl = [-5 7]; scn(3).yl = [-6 6];
scn(3).title = 'Simulink PID trio3 Hover';
scn(3).refcircle = false;

% trio3 line v=0.5 d=20, 60s
T = 60; t = (0:dtc:T)';
scn(4).name = 'trio3_line'; scn(4).births = BIRTH_TRIO3; scn(4).T = T; scn(4).t = t;
scn(4).vx = 0.5*(t>=10 & t<50); scn(4).vy = zeros(size(t)); scn(4).vz = -1.0*(t<5);
scn(4).mass = []; scn(4).xl = [-5 26]; scn(4).yl = [-6 6];
scn(4).title = 'Simulink PID trio3 Line v=0.5 m/s';
scn(4).refcircle = false;

% pair2 扰动出生 line v=0.5, 70s — PID 无法收敛
T = 70; t = (0:dtc:T)';
scn(5).name = 'pair2_perturbed'; scn(5).births = BIRTH_PAIR2_PERT; scn(5).T = T; scn(5).t = t;
scn(5).vx = 0.5*(t>=10 & t<50); scn(5).vy = zeros(size(t)); scn(5).vz = -1.0*(t<5);
scn(5).mass = []; scn(5).xl = [-5 25]; scn(5).yl = [-4 4];
scn(5).title = 'Simulink PID pair2 Line — perturbed birth (PID 不收敛)';
scn(5).refcircle = false;

% trio3 circle 扰动出生 + 质量离散, 65s
T = 65; t = (0:dtc:T)'; R = 10; v = 1.5; om = v/R; tc = max(t-10, 0);
scn(6).name = 'trio3_circle_perturbed'; scn(6).births = BIRTH_TRIO3_PERT;
scn(6).T = T; scn(6).t = t;
scn(6).vx = -v*sin(om*tc).*(t>=10); scn(6).vy = v*cos(om*tc).*(t>=10);
scn(6).vz = -1.0*(t<5);
scn(6).mass = Mass*[1.05 0.95 1.00]; scn(6).xl = [-26 10]; scn(6).yl = [-15 15];
scn(6).title = 'Simulink PID trio3 Circle — perturbed birth (PID 不收敛)';
scn(6).refcircle = true;

cols = {[0 0.447 0.741], [0.851 0.325 0.098], [0.466 0.674 0.188]};
armR = 0.8; FPS = 20; SPEED = 2.0;

for s = 1:numel(scn)
    sc = scn(s);
    n = size(sc.births, 1);
    fprintf('\n===== %s (%d 机, %ds) =====\n', sc.name, n, sc.T);
    vx_ts = timeseries(sc.vx, sc.t); vy_ts = timeseries(sc.vy, sc.t);
    vz_ts = timeseries(sc.vz, sc.t);

    XeAll = cell(n,1); tAll = [];
    for d = 1:n
        simIn = Simulink.SimulationInput(mdl);
        simIn = simIn.setBlockParameter(blk6dof, 'xme_0', mat2str(sc.births(d,:)));
        simIn = simIn.setModelParameter('StopTime', num2str(sc.T));
        simIn = simIn.setVariable('vx_ts', vx_ts);
        simIn = simIn.setVariable('vy_ts', vy_ts);
        simIn = simIn.setVariable('vz_ts', vz_ts);
        if ~isempty(sc.mass), simIn = simIn.setVariable('Mass', sc.mass(d)); end
        fprintf('  drone%d ... ', d-1); tic;
        out = sim(simIn);
        XeAll{d} = out.logsout.getElement('Xe').Values.Data;
        tAll = out.logsout.getElement('Xe').Values.Time;
        fprintf('done (%.1f s)\n', toc);
    end
    L = min(cellfun(@(x) size(x,1), XeAll));
    tAll = tAll(1:L);
    for d = 1:n, XeAll{d} = XeAll{d}(1:L,:); end

    % ── 动画渲染 ──
    vidfile = fullfile(OUTDIR, ['simulink_' sc.name '_formation.mp4']);
    stp = max(1, round(SPEED/FPS/(tAll(2)-tAll(1))));
    frames = 1:stp:L;
    TRAIL = round(15/(tAll(2)-tAll(1)));

    fig = figure('Visible', 'off', 'Position', [80 80 1280 720], 'Color', 'w');
    ax = axes(fig); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
    xlabel('X north [m]'); ylabel('Y east [m]'); zlabel('Alt [m]');
    view(ax, -35, 28); axis(ax, 'equal');
    xlim(ax, sc.xl); ylim(ax, sc.yl); zlim(ax, [0 7]);

    if sc.refcircle
        th = linspace(0, 2*pi, 200);
        plot3(ax, (3-10)+10*cos(th), 10*sin(th), 5*ones(size(th)), 'k--', 'LineWidth', 0.7);
    end

    hArm1 = gobjects(n,1); hArm2 = gobjects(n,1); hDot = gobjects(n,1); hTrail = gobjects(n,1);
    for d = 1:n
        hTrail(d) = plot3(ax, nan,nan,nan, '-', 'Color', [cols{d} 0.45], 'LineWidth', 1.0);
        hArm1(d)  = plot3(ax, nan,nan,nan, '-', 'Color', cols{d}, 'LineWidth', 2.5);
        hArm2(d)  = plot3(ax, nan,nan,nan, '-', 'Color', cols{d}, 'LineWidth', 2.5);
        hDot(d)   = plot3(ax, nan,nan,nan, 'o', 'Color', cols{d}, ...
            'MarkerFaceColor', cols{d}, 'MarkerSize', 6);
    end
    hForm = plot3(ax, nan,nan,nan, ':', 'Color', [0.4 0.4 0.4], 'LineWidth', 1.0);
    hTitle = title(ax, '');
    legend(ax, hDot, arrayfun(@(d) sprintf('d%d', d-1), 1:n, 'UniformOutput', false), ...
        'Location', 'northeast');

    vw = VideoWriter(vidfile, 'MPEG-4'); vw.FrameRate = FPS; open(vw);
    fprintf('  渲染 %d 帧 → %s\n', numel(frames), vidfile);
    for k = frames
        for d = 1:n
            p = XeAll{d}(k,:); alt = -p(3);
            set(hArm1(d), 'XData', p(1)+[-armR armR], 'YData', p(2)+[0 0], 'ZData', [alt alt]);
            set(hArm2(d), 'XData', p(1)+[0 0], 'YData', p(2)+[-armR armR], 'ZData', [alt alt]);
            set(hDot(d), 'XData', p(1), 'YData', p(2), 'ZData', alt);
            i0 = max(1, k-TRAIL);
            set(hTrail(d), 'XData', XeAll{d}(i0:k,1), 'YData', XeAll{d}(i0:k,2), ...
                'ZData', -XeAll{d}(i0:k,3));
        end
        poly = zeros(n+1, 3);
        for d = 1:n, poly(d,:) = XeAll{d}(k,:); end
        poly(n+1,:) = XeAll{1}(k,:);
        set(hForm, 'XData', poly(:,1), 'YData', poly(:,2), 'ZData', -poly(:,3));
        set(hTitle, 'String', sprintf('%s — t = %.1f s (2x)', sc.title, tAll(k)));
        writeVideo(vw, getframe(fig));
    end
    close(vw); close(fig);
    fprintf('  完成: %s\n', vidfile);
end
fprintf('\n全部完成。\n');
