function mdl = build_swarm_model(n, force)
%BUILD_SWARM_MODEL 组装 n 机闭环模型 mpc_swarm_<n>.slx
%   drone_i 子系统(uav_plant_io 全内容拷贝, Goto/From 标签局部化) × n
%   + Interpreted MATLAB Fcn 控制器(mpc_swarm_step, 50Hz)
%   + 每机 To Workspace 记录 Xe_i / Ve_i
% 出生位置由 scenario_run 在 sim 前 set_param 各 6DOF xme_0 注入。
if nargin < 2, force = false; end
here = fileparts(mfilename('fullpath'));
mdl = sprintf('mpc_swarm_%d', n);
dst = fullfile(here, [mdl '.slx']);
if exist(dst, 'file') && ~force
    if ~bdIsLoaded(mdl), load_system(dst); end
    return
end

build_plant_io(false);
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);

% ── n 个被控对象子系统 ──────────────────────────────────────────────────
if ~bdIsLoaded('uav_plant_io'), load_system(fullfile(here, 'uav_plant_io.slx')); end
for i = 1:n
    sub = sprintf('%s/drone_%d', mdl, i-1);
    add_block('built-in/Subsystem', sub, ...
        'Position', [600, 80 + 220*(i-1), 760, 200 + 220*(i-1)]);
    Simulink.BlockDiagram.copyContentsToSubsystem('uav_plant_io', sub);
end
close_system('uav_plant_io', 0);

% ── 控制器：Mux[Clock; Xe;Ve ×n] → mpc_swarm_step → Demux 3n ────────────
add_block('simulink/Sources/Clock', [mdl '/clk'], 'Position', [40 40 60 60]);
add_block('simulink/Signal Routing/Mux', [mdl '/state_mux'], ...
    'Inputs', num2str(1 + 2*n), 'Position', [120 40 130 120 + 60*n]);
add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', ...
    [mdl '/mpc_ctrl'], 'MATLABFcn', 'mpc_swarm_step(u)', ...
    'OutputDimensions', num2str(3*n), 'SampleTime', '0.02', ...
    'Position', [200 70 330 130]);
add_block('simulink/Signal Routing/Demux', [mdl '/cmd_demux'], ...
    'Outputs', num2str(3*n), 'Position', [400 40 410 120 + 80*n]);

add_line(mdl, 'clk/1', 'state_mux/1', 'autorouting', 'on');
add_line(mdl, 'state_mux/1', 'mpc_ctrl/1', 'autorouting', 'on');
add_line(mdl, 'mpc_ctrl/1', 'cmd_demux/1', 'autorouting', 'on');

for i = 1:n
    d = sprintf('drone_%d', i-1);
    % 状态回采：Xe → mux 槽 2i，Ve → 槽 2i+1
    add_line(mdl, [d '/1'], sprintf('state_mux/%d', 2*i),   'autorouting', 'on');
    add_line(mdl, [d '/2'], sprintf('state_mux/%d', 2*i+1), 'autorouting', 'on');
    % 指令下发：demux 3(i-1)+1..3 → vx/vy/vz
    for c = 1:3
        add_line(mdl, sprintf('cmd_demux/%d', 3*(i-1)+c), ...
            sprintf('%s/%d', d, c), 'autorouting', 'on');
    end
end
% To Workspace（Xe_i / Ve_i, timeseries）
for i = 1:n
    d = sprintf('drone_%d', i-1);
    twx = sprintf('%s/log_Xe_%d', mdl, i-1);
    twv = sprintf('%s/log_Ve_%d', mdl, i-1);
    add_block('simulink/Sinks/To Workspace', twx, ...
        'VariableName', sprintf('Xe_%d', i-1), 'SaveFormat', 'Timeseries', ...
        'Position', [860, 60 + 220*(i-1), 940, 90 + 220*(i-1)]);
    add_block('simulink/Sinks/To Workspace', twv, ...
        'VariableName', sprintf('Ve_%d', i-1), 'SaveFormat', 'Timeseries', ...
        'Position', [860, 110 + 220*(i-1), 940, 140 + 220*(i-1)]);
    add_line(mdl, [d '/1'], sprintf('log_Xe_%d/1', i-1), 'autorouting', 'on');
    add_line(mdl, [d '/2'], sprintf('log_Ve_%d/1', i-1), 'autorouting', 'on');
end

set_param(mdl, 'Solver', 'ode4', 'FixedStep', '0.004', ...
    'SignalLogging', 'off', 'ReturnWorkspaceOutputs', 'on');
save_system(mdl, dst);
fprintf('[swarm] %s 构建完成 (%d 机)\n', dst, n);
end
