function run_all_consensus()
%RUN_ALL_CONSENSUS 二阶一致性对照组：连跑全 12 场景（4 队形 × 3 运动）
%   写 results_consensus.csv + report/figures/cons_<队形>_<运动>.png。
%   与 MPC 同场景集；跑完用 compare_mpc_consensus 出对比表&图。
forms = {'solo1', 'pair2', 'trio3', 'cross5'};
modes = {'hover', 'line', 'circle'};
tall = tic;
for f = 1:numel(forms)
    for m = 1:numel(modes)
        scenario_run(forms{f}, modes{m}, 'consensus');
    end
end
fprintf('\n[consensus] 12 场景完成 %.1fs → results_consensus.csv\n', toc(tall));
end
