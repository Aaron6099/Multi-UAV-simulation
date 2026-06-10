% inspect_wiring.m — 确认 x/y/z, Step2/Step4, Goto/From 的接线
run('init.m');
mdl = 'simple_vel_pid_4rotors';
if bdIsLoaded(mdl), close_system(mdl, 0); end
load_system(mdl);

fprintf('===== Goto 标签 =====\n');
gotos = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'Goto');
for i = 1:numel(gotos)
    fprintf('%-12s tag=%s\n', get_param(gotos{i}, 'Name'), get_param(gotos{i}, 'GotoTag'));
end
fprintf('\n===== From 标签 =====\n');
froms = find_system(mdl, 'SearchDepth', 1, 'BlockType', 'From');
for i = 1:numel(froms)
    fprintf('%-12s tag=%s\n', get_param(froms{i}, 'Name'), get_param(froms{i}, 'GotoTag'));
end

fprintf('\n===== 顶层连线 (src → dst) =====\n');
lines = find_system(mdl, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
for i = 1:numel(lines)
    src = get_param(lines(i), 'SrcBlockHandle');
    if src ~= -1
        srcname = get_param(src, 'Name');
        dsts = get_param(lines(i), 'DstBlockHandle');
        for d = dsts'
            if d ~= -1
                fprintf('%-25s → %s\n', srcname, get_param(d, 'Name'));
            end
        end
    end
end

fprintf('\n===== pid_velocity 输入端口 =====\n');
ports = find_system([mdl '/pid_velocity'], 'SearchDepth', 1, 'BlockType', 'Inport');
for i = 1:numel(ports)
    fprintf('Inport %s: %s\n', get_param(ports{i}, 'Port'), get_param(ports{i}, 'Name'));
end
close_system(mdl, 0);
