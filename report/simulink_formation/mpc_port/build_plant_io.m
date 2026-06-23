function build_plant_io(force)
%BUILD_PLANT_IO 由母版 simple_vel_pid_4rotors 派生 IO 化被控对象 uav_plant_io.slx
%   Inport: vx_cmd / vy_cmd / vz_cmd / yaw_cmd（替换 x / y / Step2 及 pid_velocity/yaw_d）
%   Outport: Xe(=6DOF Port2) / Ve(=6DOF Port1)
% 动画注掉。原件不动（save_system 另存）。
if nargin < 1, force = false; end
here = fileparts(mfilename('fullpath'));
addpath(fileparts(here));               % 母版 slx 在上级目录
dst = fullfile(here, 'uav_plant_io.slx');
if exist(dst, 'file') && ~force, return; end

src = 'simple_vel_pid_4rotors';
mdl = 'uav_plant_io';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if bdIsLoaded(src), close_system(src, 0); end
load_system(src);
save_system(src, dst);
close_system(src, 0);
load_system(mdl);

% 指令源 → Inport（Mux4 输入 1/2/3 = vx/vy/vz）
old_srcs = {'x', 'y', 'Step2'};
names    = {'vx_cmd', 'vy_cmd', 'vz_cmd'};
for i = 1:3
    blk = [mdl '/' old_srcs{i}];
    ph = get_param(blk, 'PortHandles');
    ln = get_param(ph.Outport(1), 'Line');
    if ln ~= -1, delete_line(ln); end
    delete_block(blk);
    add_block('simulink/Sources/In1', [mdl '/' names{i}], ...
        'Port', num2str(i), 'Position', [40, 60+70*i, 70, 75+70*i]);
    add_line(mdl, [names{i} '/1'], sprintf('Mux4/%d', i), 'autorouting', 'on');
end

% yaw_cmd → Inport 4，替换 pid_velocity/yaw_d（原为子系统内部 Inport，
% 外层 Mux4 只有3路，yaw_d 直接从顶层穿进去）
yaw_blk = [mdl '/pid_velocity/yaw_d'];
ph_yaw = get_param(yaw_blk, 'PortHandles');
ln_yaw = get_param(ph_yaw.Outport(1), 'Line');
if ln_yaw ~= -1, delete_line(ln_yaw); end
delete_block(yaw_blk);
% 在顶层加 yaw_cmd Inport，通过 Goto/From 穿入子系统
add_block('simulink/Sources/In1', [mdl '/yaw_cmd'], ...
    'Port', '4', 'Position', [40, 290, 70, 305]);
add_block('simulink/Signal Routing/Goto', [mdl '/Goto_yaw'], ...
    'GotoTag', [mdl '_yaw_cmd'], 'Position', [120, 290, 200, 305]);
add_line(mdl, 'yaw_cmd/1', 'Goto_yaw/1', 'autorouting', 'on');
add_block('simulink/Signal Routing/From', [mdl '/pid_velocity/yaw_d'], ...
    'GotoTag', [mdl '_yaw_cmd'], 'Position', [40, 200, 120, 215]);

% 6DOF Port2=Xe, Port1=Ve → Outport（在既有连线上分支）
blk6dof = [mdl '/6DOF (Euler Angles)'];
add_block('simulink/Sinks/Out1', [mdl '/Xe_out'], 'Port', '1', ...
    'Position', [900, 100, 930, 115]);
add_block('simulink/Sinks/Out1', [mdl '/Ve_out'], 'Port', '2', ...
    'Position', [900, 160, 930, 175]);
ph6 = get_param(blk6dof, 'PortHandles');
add_line(mdl, ph6.Outport(2), get_param([mdl '/Xe_out'], 'PortHandles').Inport(1), ...
    'autorouting', 'on');
add_line(mdl, ph6.Outport(1), get_param([mdl '/Ve_out'], 'PortHandles').Inport(1), ...
    'autorouting', 'on');

% 注掉动画（批量仿真提速 + batch 无图形环境）
try, set_param([mdl '/UAV Animation'], 'Commented', 'on'); catch, end

set_param(mdl, 'Solver', 'ode4', 'FixedStep', '0.004');
save_system(mdl);
close_system(mdl, 0);
fprintf('[plant] %s 构建完成\n', dst);
end
