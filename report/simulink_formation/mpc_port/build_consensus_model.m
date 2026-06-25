function mdl = build_consensus_model(n, force)
%BUILD_CONSENSUS_MODEL 组装 n 机二阶一致性闭环模型 consensus_swarm_<n>.slx
%   结构镜像 build_swarm_model，仅把控制器块换成 consensus_swarm_step；
%   复用同一被控对象 uav_plant_io.slx（与 MPC 同植物 → 公平对比）。
%   drone_i 子系统 × n + Interpreted MATLAB Fcn(consensus_swarm_step, 50Hz)
%   + 每机 To Workspace 记录 Xe_i / Ve_i。出生位置由 scenario_run set_param 注入。
if nargin < 2, force = false; end
here = fileparts(mfilename('fullpath'));
mdl = sprintf('consensus_swarm_%d', n);
dst = fullfile(here, [mdl '.slx']);
if exist(dst, 'file') && ~force
    if ~bdIsLoaded(mdl), load_system(dst); end
    return
end

build_plant_io(false);
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);

% ── n 个被控对象子系统（与 MPC 同植物 uav_plant_io）─────────────────────
if ~bdIsLoaded('uav_plant_io'), load_system(fullfile(here, 'uav_plant_io.slx')); end
for i = 1:n
    sub = sprintf('%s/drone_%d', mdl, i-1);
    add_block('built-in/Subsystem', sub, ...
        'Position', [600, 80 + 220*(i-1), 760, 200 + 220*(i-1)]);
    Simulink.BlockDiagram.copyContentsToSubsystem('uav_plant_io', sub);
end
close_system('uav_plant_io', 0);

% ── 控制器：Mux[Clock; Xe;Ve ×n] → consensus_swarm_step → Demux 4n ───────
add_block('simulink/Sources/Clock', [mdl '/clk'], 'Position', [40 40 60 60]);
add_block('simulink/Signal Routing/Mux', [mdl '/state_mux'], ...
    'Inputs', num2str(1 + 2*n), 'Position', [120 40 130 120 + 60*n]);
add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', ...
    [mdl '/cons_ctrl'], 'MATLABFcn', 'consensus_swarm_step(u)', ...
    'OutputDimensions', num2str(4*n), 'SampleTime', '0.02', ...
    'Position', [200 70 330 130]);
add_block('simulink/Signal Routing/Demux', [mdl '/cmd_demux'], ...
    'Outputs', num2str(4*n), 'Position', [400 40 410 120 + 80*n]);

add_line(mdl, 'clk/1', 'state_mux/1', 'autorouting', 'on');
add_line(mdl, 'state_mux/1', 'cons_ctrl/1', 'autorouting', 'on');
add_line(mdl, 'cons_ctrl/1', 'cmd_demux/1', 'autorouting', 'on');

for i = 1:n
    d = sprintf('drone_%d', i-1);
    % 状态回采：Xe → mux 槽 2i，Ve → 槽 2i+1
    add_line(mdl, [d '/1'], sprintf('state_mux/%d', 2*i),   'autorouting', 'on');
    add_line(mdl, [d '/2'], sprintf('state_mux/%d', 2*i+1), 'autorouting', 'on');
    % 指令下发：demux 4(i-1)+1..4 → vx/vy/vz/yaw
    for c = 1:4
        add_line(mdl, sprintf('cmd_demux/%d', 4*(i-1)+c), ...
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
fprintf('[consensus] %s 构建完成 (%d 机)\n', dst, n);
end
