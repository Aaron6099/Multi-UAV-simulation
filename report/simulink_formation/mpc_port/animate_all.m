%% animate_all.m — 12 个场景全部出动画(+PNG+results.csv 去重追加)
% 逐个调 animate_mpc：弹窗实时播放 + 另存 ../figures/mpc_<f>_<m>_anim.mp4。
% 幂等：PNG 确定性重生成、results.csv 已有行自动跳过。
% 全部 12 个约 15~18 min(每场景仿真 ~75-80s + 渲染；cross5 circle T=93s 较长)。
% 只想补缺动画的场景：把下面 combos 改成对应行即可。
combos = { ...
    'solo1','hover'; 'solo1','line'; 'solo1','circle'; ...
    'pair2','hover'; 'pair2','line'; 'pair2','circle'; ...
    'trio3','hover'; 'trio3','line'; 'trio3','circle'; ...
    'cross5','hover'; 'cross5','line'; 'cross5','circle'; ...
    'grid9','hover'; 'grid9','line'; 'grid9','circle'};
addpath(fileparts(mfilename('fullpath')));
for k = 1:size(combos,1)
    fprintf('\n========== [%d/%d] %s %s ==========\n', k, size(combos,1), combos{k,:});
    animate_mpc(combos{k,1}, combos{k,2});
end
fprintf('\n全部 %d 个动画完成（solo1×3 + pair2×3 + trio3×3 + cross5×3 + grid9×3）。\n', size(combos,1));
