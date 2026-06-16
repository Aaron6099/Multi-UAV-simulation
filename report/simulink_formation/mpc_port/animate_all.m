%% animate_all.m — 9 个场景全部出动画(+PNG+results.csv 去重追加)
% 逐个调 animate_mpc：弹窗实时播放 + 另存 ../figures/mpc_<f>_<m>_anim.mp4。
% 幂等：PNG 确定性重生成、results.csv 已有行自动跳过。
% 全部 9 个约 11~13 min(每场景仿真 ~75-80s + 渲染)。
% 只想补缺动画的 7 个：把下面 combos 改成那 7 行即可。
combos = { ...
    'solo1','hover'; 'solo1','line'; 'solo1','circle'; ...
    'pair2','hover'; 'pair2','line'; 'pair2','circle'; ...
    'trio3','hover'; 'trio3','line'; 'trio3','circle'};
addpath(fileparts(mfilename('fullpath')));
for k = 1:size(combos,1)
    fprintf('\n========== [%d/%d] %s %s ==========\n', k, size(combos,1), combos{k,:});
    animate_mpc(combos{k,1}, combos{k,2});
end
fprintf('\n全部 %d 个动画完成。\n', size(combos,1));
