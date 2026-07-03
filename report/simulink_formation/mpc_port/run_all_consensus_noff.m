function run_all_consensus_noff()
%RUN_ALL_CONSENSUS_NOFF 公平基线：去 leader 速度前馈的二阶一致性，连跑全 15 场景
%   写 results_consensus_noff.csv + cons_noff_*.png。与带前馈版唯一差别 = ff=false
%   （ablation：隔离"前馈"这一项对 track_err 的贡献）。
forms = {'solo1', 'pair2', 'trio3', 'cross5', 'grid9'};
modes = {'hover', 'line', 'circle'};
tall = tic;
for f = 1:numel(forms)
    for m = 1:numel(modes)
        scenario_run(forms{f}, modes{m}, 'consensus', false);
    end
end
fprintf('\n[consensus-noff] 12 场景完成 %.1fs → results_consensus_noff.csv\n', toc(tall));
end
