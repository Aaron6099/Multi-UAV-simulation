% inspect_model.m — 探查 simple_vel_pid_4rotors 的指令源与顶层结构
run('init.m');
mdl = 'simple_vel_pid_4rotors';
if bdIsLoaded(mdl), close_system(mdl, 0); end
load_system(mdl);

fprintf('===== 顶层块列表 =====\n');
blks = find_system(mdl, 'SearchDepth', 1);
for i = 2:numel(blks)
    fprintf('%-50s  %s\n', get_param(blks{i}, 'Name'), get_param(blks{i}, 'BlockType'));
end

fprintf('\n===== Step / Constant / Signal Builder / From Workspace 块 =====\n');
types = {'Step', 'Constant', 'FromWorkspace', 'SignalBuilder', 'Ramp', 'Sin'};
for k = 1:numel(types)
    found = find_system(mdl, 'BlockType', types{k});
    for i = 1:numel(found)
        fprintf('[%s] %s\n', types{k}, found{i});
        if strcmp(types{k}, 'Step')
            fprintf('    Time=%s  Before=%s  After=%s\n', ...
                get_param(found{i}, 'Time'), ...
                get_param(found{i}, 'Before'), get_param(found{i}, 'After'));
        elseif strcmp(types{k}, 'Constant')
            fprintf('    Value=%s\n', get_param(found{i}, 'Value'));
        end
    end
end

fprintf('\n===== 6DOF 初始条件 =====\n');
blk = [mdl '/6DOF (Euler Angles)'];
try
    fprintf('Xe0=%s\nVe0(body)=%s\nEuler0=%s\n', ...
        get_param(blk, 'xme_0'), get_param(blk, 'Vm_0'), get_param(blk, 'eul_0'));
catch ME
    fprintf('读取失败: %s\n', ME.message);
    % 列出全部 dialog 参数名
    dp = get_param(blk, 'DialogParameters');
    disp(fieldnames(dp));
end
close_system(mdl, 0);
