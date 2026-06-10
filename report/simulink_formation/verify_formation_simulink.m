%% verify_formation_simulink.m — Simulink PID 2机/3机编队多模式验证
% 基于 simple_vel_pid_4rotors.slx（单机 6DOF+串级PID），按编队出生点
% 多实例运行：每机注入同一速度指令轨迹（与实机 MPC→PX4 速度通道同构），
% 不同 Xe0 出生位置，合并轨迹计算队形误差 / 最小间距。
%
% 场景: pair2 hover / pair2 line(v=0.5) / trio3 circle(R=10,v=1.5)
% 输出: ../figures/simulink_pair2_hover.png / _pair2_line.png / _trio3_circle.png
%       控制台指标 + verdict

clear; clc; close all;
run('init.m');

OUTDIR = fullfile(fileparts(pwd), 'figures');   % report/figures
if ~exist(OUTDIR, 'dir'), mkdir(OUTDIR); end

%% ── 1. 构建多机版模型（速度指令 From Workspace 化，去动画）────────────────
src = 'simple_vel_pid_4rotors';
mdl = 'pid_formation_multi';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if bdIsLoaded(src), close_system(src, 0); end
load_system(src);
save_system(src, [mdl '.slx']);   % 复制为新模型，不动原件
close_system(src, 0);
load_system(mdl);

% 替换 Mux4 三路输入: x→vx_ts, y→vy_ts, Step2→vz_ts
old_srcs = {'x', 'y', 'Step2'};
new_vars = {'vx_ts', 'vy_ts', 'vz_ts'};
for i = 1:3
    blk = [mdl '/' old_srcs{i}];
    ph = get_param(blk, 'PortHandles');
    ln = get_param(ph.Outport(1), 'Line');
    if ln ~= -1, delete_line(ln); end
    nb = [mdl '/cmd_' new_vars{i}];
    add_block('simulink/Sources/From Workspace', nb, ...
        'VariableName', new_vars{i}, ...
        'SampleTime', '0', 'Interpolate', 'on', ...
        'Position', [50, 100+80*i, 130, 130+80*i]);
    add_line(mdl, ['cmd_' new_vars{i} '/1'], sprintf('Mux4/%d', i), 'autorouting', 'on');
end

% 关闭动画（多次批量跑提速）
try, set_param([mdl '/UAV Animation'], 'Commented', 'on'); catch, end

% 信号记录: Xe (6DOF Port 2)
set_param(mdl, 'SignalLogging', 'on', 'SignalLoggingName', 'logsout', ...
    'SignalLoggingSaveFormat', 'Dataset');
blk6dof = [mdl '/6DOF (Euler Angles)'];
ph6 = get_param(blk6dof, 'PortHandles');
set_param(ph6.Outport(2), 'DataLogging', 'on', ...
    'DataLoggingNameMode', 'Custom', 'DataLoggingName', 'Xe');

set_param(mdl, 'Solver', 'ode4', 'FixedStep', num2str(SampleTime));
save_system(mdl);
fprintf('[模型] %s.slx 构建完成（速度指令 From Workspace 注入 + Xe 记录）\n', mdl);

%% ── 2. 场景定义 ─────────────────────────────────────────────────────────
% 出生点 NED (与 scenarios.yaml formations 一致)，z=0 地面
BIRTH_PAIR2 = [ 0     0      0;
               -3     0      0];
BIRTH_TRIO3 = [ 3     0      0;
               -1.5   2.598  0;
               -1.5  -2.598  0];

% 速度指令轨迹生成 (全员同指令；爬升 5s @1m/s → 悬停 5m，对应 target_alt)
dtc = 0.05;
scn = struct([]);

% S-A: pair2 hover, 30s
T = 30; t = (0:dtc:T)';
scn(1).name = 'pair2-hover'; scn(1).births = BIRTH_PAIR2; scn(1).T = T;
scn(1).vx = zeros(size(t)); scn(1).vy = zeros(size(t));
scn(1).vz = -1.0 * (t < 5); scn(1).t = t;
scn(1).png = 'simulink_pair2_hover.png';
scn(1).title = 'Simulink PID pair2 Hover (climb to 5 m, hold)';

% S-B: pair2 line v=0.5, d=20 → 巡航 40s, 总 60s
T = 60; t = (0:dtc:T)';
scn(2).name = 'pair2-line'; scn(2).births = BIRTH_PAIR2; scn(2).T = T;
scn(2).vx = 0.5 * (t >= 10 & t < 50);
scn(2).vy = zeros(size(t));
scn(2).vz = -1.0 * (t < 5); scn(2).t = t;
scn(2).png = 'simulink_pair2_line.png';
scn(2).title = 'Simulink PID pair2 Line v=0.5 m/s, d=20 m';

% S-C: trio3 circle R=10 v=1.5 → 一整圈 41.9s, 总 65s
T = 65; t = (0:dtc:T)';
R = 10; v = 1.5; om = v / R; tc = max(t - 10, 0);
scn(3).name = 'trio3-circle'; scn(3).births = BIRTH_TRIO3; scn(3).T = T;
scn(3).vx = -v * sin(om * tc) .* (t >= 10);
scn(3).vy =  v * cos(om * tc) .* (t >= 10);
scn(3).vz = -1.0 * (t < 5); scn(3).t = t;
scn(3).png = 'simulink_trio3_circle.png';
scn(3).title = 'Simulink PID trio3 Circle R=10 m, v=1.5 m/s';

D_SAFE = 1.5; FORM_THR = 0.5;

%% ── 3. 逐场景逐机仿真 ────────────────────────────────────────────────────
results = cell(numel(scn), 1);
for s = 1:numel(scn)
    sc = scn(s);
    n = size(sc.births, 1);
    fprintf('\n========= %s (%d 机, %ds) =========\n', sc.name, n, sc.T);

    vx_ts = timeseries(sc.vx, sc.t);
    vy_ts = timeseries(sc.vy, sc.t);
    vz_ts = timeseries(sc.vz, sc.t);

    XeAll = cell(n, 1); tAll = [];
    for d = 1:n
        birth = sc.births(d, :);
        simIn = Simulink.SimulationInput(mdl);
        simIn = simIn.setBlockParameter(blk6dof, 'xme_0', mat2str(birth));
        simIn = simIn.setModelParameter('StopTime', num2str(sc.T));
        simIn = simIn.setVariable('vx_ts', vx_ts);
        simIn = simIn.setVariable('vy_ts', vy_ts);
        simIn = simIn.setVariable('vz_ts', vz_ts);
        fprintf('  drone%d @[%g %g %g] ... ', d-1, birth);
        tic; out = sim(simIn); el = toc;
        Xe_sig = out.logsout.getElement('Xe');
        XeAll{d} = Xe_sig.Values.Data;      % Nx3 NED
        tAll = Xe_sig.Values.Time;
        fprintf('done (%.1f s)\n', el);
    end

    % 重采样到统一时基（ode4 定步长应已一致，保险起见取最短）
    L = min(cellfun(@(x) size(x, 1), XeAll));
    tAll = tAll(1:L);
    for d = 1:n, XeAll{d} = XeAll{d}(1:L, :); end

    % 队形误差: 相对 drone0 的偏差 vs 标称偏移
    form_err = zeros(L, 1);
    for d = 2:n
        nominal = sc.births(d, :) - sc.births(1, :);
        rel = XeAll{d} - XeAll{1};
        e = sqrt(sum((rel(:, 1:2) - nominal(1:2)).^2, 2));   % 水平
        form_err = max(form_err, e);
    end
    % 最小间距
    min_sp = inf(L, 1);
    for a = 1:n
        for b = a+1:n
            dab = sqrt(sum((XeAll{a} - XeAll{b}).^2, 2));
            min_sp = min(min_sp, dab);
        end
    end

    half = floor(L/2);
    fe_mean = mean(form_err(half:end)); fe_max = max(form_err);
    sp_min = min(min_sp);
    ok = (fe_mean < FORM_THR) && (sp_min >= D_SAFE);
    fprintf('  form_err 稳态均值=%.3f m  全程峰值=%.3f m\n', fe_mean, fe_max);
    fprintf('  min_spacing=%.3f m (d_safe=%.1f)\n', sp_min, D_SAFE);
    if ok, fprintf('  VERDICT: PASS\n'); else, fprintf('  VERDICT: REVIEW\n'); end

    % ── 绘图: 4 联 ──
    cols = {[0 0.447 0.741], [0.851 0.325 0.098], [0.466 0.674 0.188]};
    fig = figure('Visible', 'off', 'Position', [50 50 1300 850]);

    subplot(2,2,1); hold on;
    for d = 1:n
        plot(XeAll{d}(:,1), XeAll{d}(:,2), 'Color', cols{d}, 'LineWidth', 1.2);
        plot(XeAll{d}(1,1),  XeAll{d}(1,2),  'o', 'Color', cols{d}, 'MarkerFaceColor', cols{d});
        plot(XeAll{d}(end,1),XeAll{d}(end,2),'s', 'Color', cols{d}, 'MarkerFaceColor', cols{d});
    end
    xlabel('X north [m]'); ylabel('Y east [m]'); title('XY Trajectory (NED)');
    axis equal; grid on;
    legend(arrayfun(@(d) sprintf('d%d', d-1), 1:n, 'UniformOutput', false), 'Location', 'best');

    subplot(2,2,2); hold on;
    for d = 1:n
        plot(tAll, -XeAll{d}(:,3), 'Color', cols{d}, 'LineWidth', 1.2);
    end
    yline(5, 'k--'); xlabel('t [s]'); ylabel('Altitude [m]');
    title('Altitude (target 5 m)'); grid on;

    subplot(2,2,3);
    plot(tAll, form_err, 'r', 'LineWidth', 1.2); hold on;
    yline(FORM_THR, 'k--', '0.5 m');
    xlabel('t [s]'); ylabel('error [m]'); title('Formation Max Error (rel. d0)'); grid on;

    subplot(2,2,4);
    plot(tAll, min_sp, 'Color', [0.2 0.6 0.2], 'LineWidth', 1.2); hold on;
    yline(D_SAFE, 'r--', 'd\_safe 1.5 m');
    xlabel('t [s]'); ylabel('spacing [m]'); title('Min Inter-drone Spacing'); grid on;

    sgtitle(sc.title, 'FontSize', 12, 'FontWeight', 'bold');
    png = fullfile(OUTDIR, sc.png);
    saveas(fig, png); close(fig);
    fprintf('  [图] %s\n', png);

    results{s} = struct('name', sc.name, 'fe_mean', fe_mean, 'fe_max', fe_max, ...
        'sp_min', sp_min, 'ok', ok);
end

%% ── 4. 汇总 ─────────────────────────────────────────────────────────────
fprintf('\n================== SUMMARY ==================\n');
fprintf('%-15s %12s %12s %12s  %s\n', 'scenario', 'form_mean[m]', 'form_max[m]', 'min_sp[m]', 'verdict');
for s = 1:numel(results)
    r = results{s};
    v = 'REVIEW'; if r.ok, v = 'PASS'; end
    fprintf('%-15s %12.3f %12.3f %12.3f  %s\n', r.name, r.fe_mean, r.fe_max, r.sp_min, v);
end
close_system(mdl, 0);
fprintf('\n完成。图已存 %s\n', OUTDIR);
