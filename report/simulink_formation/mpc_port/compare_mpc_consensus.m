function T = compare_mpc_consensus()
%COMPARE_MPC_CONSENSUS 合并 MPC 与二阶一致性对照组指标 → 对比表 + 分组柱状图
%   读 results.csv（MPC）与 results_consensus.csv（一致性），按 队形+运动 对齐，
%   打印对比表、存 compare_mpc_vs_consensus.csv，并出 track_err / form_err 的
%   MPC vs 一致性 分组柱状图 report/figures/compare_mpc_vs_consensus.png。
%   前置：先各自跑完 12 场景（MPC 用 animate_all/run_mpc_*；一致性用 run_all_consensus）。
here = fileparts(mfilename('fullpath'));
fm = fullfile(here, 'results.csv');
fc = fullfile(here, 'results_consensus.csv');
assert(exist(fm, 'file') == 2, '缺 results.csv：先跑 MPC（animate_all 或 run_mpc_*）');
assert(exist(fc, 'file') == 2, '缺 results_consensus.csv：先跑 run_all_consensus');
M = readtable(fm, 'TextType', 'string');
C = readtable(fc, 'TextType', 'string');
Mkey = M.formation + "_" + M.mode;
Ckey = C.formation + "_" + C.mode;

T = table('Size', [0 8], ...
    'VariableTypes', {'string','string','double','double','double','double','string','string'}, ...
    'VariableNames', {'formation','mode','form_mpc','form_cons', ...
                      'track_mpc','track_cons','vd_mpc','vd_cons'});
for r = 1:height(M)
    k = find(Ckey == Mkey(r), 1);
    if isempty(k), continue; end
    T = [T; {M.formation(r), M.mode(r), M.form_err_m(r), C.form_err_m(k), ...
             M.track_err_m(r), C.track_err_m(k), M.verdict(r), C.verdict(k)}]; %#ok<AGROW>
end
disp(T);
writetable(T, fullfile(here, 'compare_mpc_vs_consensus.csv'));

% ── 分组柱状图 ───────────────────────────────────────────────────────────
try
    lbl = T.formation + "-" + T.mode;
    fig = figure('Visible', 'off', 'Position', [50 50 1280 760]);
    subplot(2,1,1); bar([T.track_mpc T.track_cons]); grid on
    set(gca, 'XTick', 1:height(T), 'XTickLabel', lbl, 'XTickLabelRotation', 30);
    ylabel('track\_err [m]'); legend('MPC', '二阶一致性', 'Location', 'best');
    title('跟踪误差对比：MPC vs 二阶一致性对照组');
    subplot(2,1,2); bar([T.form_mpc T.form_cons]); grid on
    set(gca, 'XTick', 1:height(T), 'XTickLabel', lbl, 'XTickLabelRotation', 30);
    ylabel('form\_err [m]'); legend('MPC', '二阶一致性', 'Location', 'best');
    title('编队误差对比（邻居对均值）');
    fig_dir = fullfile(fileparts(fileparts(here)), 'figures');
    if ~exist(fig_dir, 'dir'), mkdir(fig_dir); end
    png = fullfile(fig_dir, 'compare_mpc_vs_consensus.png');
    exportgraphics(fig, png, 'Resolution', 120); close(fig);
    fprintf('对比表 → compare_mpc_vs_consensus.csv\n图 → %s\n', png);
catch ME
    fprintf('[警告] 对比图生成失败: %s\n', ME.message);
end
end
