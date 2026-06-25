function T = compare_all()
%COMPARE_ALL 三方对比：MPC vs 一致性(带前馈) vs 一致性(公平基线/去前馈)
%   读 results.csv / results_consensus.csv / results_consensus_noff.csv，按 队形+运动
%   对齐 track_err + verdict，打印表、存 compare_all.csv，出 track_err 三方分组柱状图。
%   前置：MPC（animate_all/run_mpc_*）、run_all_consensus、run_all_consensus_noff 各跑完。
here = fileparts(mfilename('fullpath'));
f0 = fullfile(here, 'results.csv');
f1 = fullfile(here, 'results_consensus.csv');
f2 = fullfile(here, 'results_consensus_noff.csv');
assert(exist(f0,'file')==2, '缺 results.csv（MPC）');
assert(exist(f1,'file')==2, '缺 results_consensus.csv（run_all_consensus）');
assert(exist(f2,'file')==2, '缺 results_consensus_noff.csv（run_all_consensus_noff）');
M = readtable(f0,'TextType','string');
A = readtable(f1,'TextType','string');
B = readtable(f2,'TextType','string');
kM = M.formation+"_"+M.mode; kA = A.formation+"_"+A.mode; kB = B.formation+"_"+B.mode;

T = table('Size',[0 8], ...
    'VariableTypes',{'string','string','double','double','double','string','string','string'}, ...
    'VariableNames',{'formation','mode','track_mpc','track_consFF','track_consNoFF', ...
                     'vd_mpc','vd_consFF','vd_consNoFF'});
for r = 1:height(M)
    ia = find(kA==kM(r),1); ib = find(kB==kM(r),1);
    if isempty(ia)||isempty(ib), continue; end
    T = [T; {M.formation(r), M.mode(r), M.track_err_m(r), A.track_err_m(ia), B.track_err_m(ib), ...
             M.verdict(r), A.verdict(ia), B.verdict(ib)}]; %#ok<AGROW>
end
disp(T);
writetable(T, fullfile(here, 'compare_all.csv'));

try
    lbl = T.formation+"-"+T.mode;
    fig = figure('Visible','off','Position',[50 50 1320 720]);
    bar([T.track_mpc T.track_consFF T.track_consNoFF]); grid on
    set(gca,'XTick',1:height(T),'XTickLabel',lbl,'XTickLabelRotation',30);
    ylabel('track\_err [m]');
    legend('MPC','一致性(带前馈)','一致性(去前馈公平基线)','Location','best');
    title('跟踪误差三方对比：MPC vs 一致性(带前馈) vs 一致性(去前馈)');
    fig_dir = fullfile(fileparts(fileparts(here)),'figures');
    if ~exist(fig_dir,'dir'), mkdir(fig_dir); end
    png = fullfile(fig_dir,'compare_all.png');
    exportgraphics(fig, png, 'Resolution', 120); close(fig);
    fprintf('三方对比表 → compare_all.csv\n图 → %s\n', png);
catch ME
    fprintf('[警告] 三方图失败: %s\n', ME.message);
end
end
