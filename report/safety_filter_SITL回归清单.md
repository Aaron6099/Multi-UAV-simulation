# safety_filter SITL 回归测试清单

> 目的：真机前在 PX4-Gazebo SITL 验证 companion 安全层 `safety_filter.py` ——
> **既要确认它没帮倒忙（正常飞行零误触发），也要确认每道保护真能触发、且 RELINQUISH→PX4 接管链路通**。
> ⚠️ Windows 只做了 `py_compile` + 模块自测 10/10，**运行时（import 解析 / ROS 集成 / 误触发 / 接管链）全未验**。此清单全部通过前，安全层不得上真机——配置不当它可能误触发、反而致险。

## 观测手段（先理解怎么看）
- 安全状态**只在节点 stdout 日志**：`safety_filter ON: ...`（启动）、`SAFETY <state> <reasons>`（1Hz，DEGRADED/HOLD 时）、`SAFETY RELINQUISH ... — 交还 PX4`（触发交还）。
- ⚠️ **health 话题/`diag_monitor` CSV 不含 safety 状态**（刻意没改 health 格式，避免破坏 diag_monitor 解析）。要量化就 grep 日志 + 配合 `analyze_flight.py` 看轨迹/间距/track_err。
- 建议每场景：`diag_monitor.py --log` 存 CSV + 节点日志重定向到文件，事后 `analyze_flight.py <csv> --plot`。
- **参数注入注意**：`safety_*` 参数在 `mpc_node.__init__` 里**构造 SafetyFilter 时读一次**，运行时 `ros2 param set` **不会重建滤波器、不生效**。改阈值须经①`scenarios.yaml` defaults（推荐，随 common 下发，**改后必 colcon build**）或②临时改 `mpc_node.py` declare_parameter 默认值 + build。

---

## Phase 0 — 不回归（最重要：确认没帮倒忙）
正常飞行 safety 必须**全程 NORMAL、零误触发**，且开/关 safety 的轨迹/verdict 基本一致。

- [ ] `colcon build` 通过 —— **这是 Windows 没法验的第一关：确认 `from mpc_control.safety_filter import SafetyFilter` 解析、setup.py 把模块打进了 install 副本**
- [ ] 启动任一场景，日志出现 `safety_filter ON: track<... alt[...] d_emerg=... d_warn=...`（确认参数如预期、滤波器真的建起来了）
- [ ] 逐场景跑 `solo1/pair2/trio3 × hover/line/circle`，全程 **无 `SAFETY` 日志**（NORMAL 不打印）。出现任何 DEGRADED/HOLD = 误触发，记下 reason
- [ ] **🔴 头号风险：飞散围栏误触发**。`max_track_dist` 默认 **5.0m**，而既有数据 **solo1 circle 的 track_err 峰值约 4.5m**（Simulink 端），余量仅 0.5m。SITL 实测各场景正常 track_err 峰值（`analyze_flight.py` 或日志 track-err），**要求 `max_track_dist ≥ 1.5 × 实测峰值`**；circle 系列尤其要确认不被瞬态顶穿。否则圆周一加速就误触发围栏→无谓 HOLD/RELINQUISH
- [ ] **碰撞地板余量**：实测各场景 min spacing 谷值 > `d_warn`（当前 `d_warn=max(d_emergency+0.5, d_safe)`）。重点看**编队成型/起飞瞬态**两机最近时刻是否跌破 d_warn（cross5 中心机、grid9 最密处、扰动出生场景）
- [ ] **jerk 限制不拖累跟踪**：对比开/关 safety 的同场景 track_err 曲线，应基本重合（`max_accel*dt*slack≈6m/s²` > max_accel，正常不该 binding；若跟踪明显变滞后→slack 调大或排查）
- [ ] **A/B 对照**：`safety_filter_enable:=false` 跑一遍同场景做基线 → 再 `true` 跑 → 轨迹/verdict/成型时间无实质差异 = safety 不改变正常行为 ✅

判据：**Phase 0 全绿才进 Phase 1**。任何正常场景误触发，先标定参数（Phase 3）再继续。

---

## Phase 1 — 各保护单独触发（降阈值/诱发，确认真起作用）

| 保护 | 诱发方法 | 期望行为 | 观测 |
|---|---|---|---|
| 飞散围栏 | `safety_max_track_dist=1.0`（yaml defaults+build）跑 line/circle | track 超 1m → 刹掉"扩大偏差"分量；持续 → DEGRADED→HOLD | 日志 `flyaway_*`；轨迹被拽住不外飞 |
| 硬碰撞地板（退化验证） | `safety_d_emergency` 设到**略高于该队形 min spacing**（如 trio3 间距 5.2→设 5.5）| 横向全停（`collide_emerg`）、两机不再靠近 | 日志 `collide_emerg(d=...)`；间距不再缩小 |
| 硬碰撞地板（真实验证） | 用扰动出生/缩小 offsets 场景（如 `offsets_scale` 调小或 S11 系列）制造真实接近 | 接近到 d_warn 抑制接近分量、到 d_emergency 全停 | `collide_warn`→`collide_emerg`；min spacing 守住不穿 |
| 估计健康门 | 飞行中 `pkill MicroXRCEAgent`（断本机位置流）或暂停 FC 位置输出 | est_ok=False → 零速 HOLD；恢复且在 relinquish 窗内 → 回 NORMAL，超窗 → RELINQUISH | 日志 `est_unhealthy`；恢复行为 |
| 失效状态机 + **RELINQUISH→PX4** | 持续越界（`safety_max_track_dist=1.0` 跑 line，让它一直违规） | DEGRADED→HOLD→（持续 `relinquish_frames`≈1s）→ **RELINQUISH，节点停发 setpoint** → **PX4 触发 offboard 失联失效保护** | 日志 `SAFETY RELINQUISH`；**QGC 里飞机切到 Hold/Land**（= 整条交还链通） |
| jerk 限制 | leader 速度突变（或临时 `max_accel` 调小）| 下发速度平滑、无单帧阶跃 | 速度曲线无突跳 |

- [ ] 飞散围栏：刹停 + 升级 HOLD 验证
- [ ] 碰撞地板：退化 + 真实两法各验一次，间距守住
- [ ] 估计门：断位置流 → HOLD；恢复行为符合预期
- [ ] **🔴 RELINQUISH→PX4 接管：节点停发后，QGC 确认 PX4 真接管（Hold/Land）**——这一项失败=安全层"交还"是空的，最危险
- [ ] jerk：突变平滑
- [ ] 记录：RELINQUISH 是 **sticky（节点不自恢复）**，恢复需重启 mpc_node / 飞手 RC 重新 OFFBOARD —— 确认这符合预期、并写进操作手册

> ⚠️ RELINQUISH 测试前必须先按 `真机安全配置清单_FC.md` 配好 `COM_OF_LOSS_T` / `COM_OBL_RC_ACT`，否则 PX4 不会接管，无法验证这条链。

---

## Phase 2 — 与既有降级路径不冲突
- [ ] acados 求解失败兜底（status≠0/异常）仍走 hover、不被 safety 干扰；无死锁
- [ ] 无 leader / 未收到自身位置 / startup 三个 hover 兜底仍正常
- [ ] `_hover_active` 语义变化已知：safety 处于非 NORMAL 时也会置 `_hover_active=True`（仅诊断用，DEGRADED 仍下发缩放速度而非悬停）——确认 `diag_monitor` 不报错、health 6 字段照常
- [ ] 多机：每机独立 safety 实例互不串扰；一机 RELINQUISH 不误连累他机

---

## Phase 3 — 标定生产参数（基于 Phase 0/1 实测回填）
- [ ] `safety_max_track_dist` = 1.5 × 正常峰值 track_err（circle 系列实测为准；预计需从 5.0 上调，如 7~8m）
- [ ] `safety_d_emergency` < d_safe（真机 d_safe=2.5 → 设 ~1.8）；确认 `d_warn` 介于 d_emergency 与正常 min spacing 之间
- [ ] `safety_min_alt`/`safety_max_alt` = 任务高度包络（target_alt=5m → 如 [0.3, 8]）
- [ ] `relinquish_frames`：瞬态 blip 不 relinquish、真故障 relinquish（按 Phase 1 观测微调）
- [ ] 测试用的降阈值/退化设置**全部还原**；把生产值写回 `scenarios.yaml` defaults（并经 `real_hardware_launch.py` conservative 取更保守一侧）
- [ ] companion `max_track_dist` 与 FC `GF_MAX_HOR_DIST` 协调：companion 软网（先刹）应在 FC 硬围栏（兜底）之内

---

## 放飞门槛（gate to flight）
- [ ] Phase 0 零误触发 + A/B 正常 verdict 不变
- [ ] Phase 1 每道保护可触发且行为正确
- [ ] **RELINQUISH→PX4 接管链实测通过**（QGC 见接管）
- [ ] Phase 3 参数标定完成并写回配置
- [ ] 渐进放飞仍从 solo1 低空起（safety 开），逐级上 pair2/trio3

---
**关联**：`safety_filter.py`、`mpc_control/mpc_node.py`（7 处接入）、`真机安全配置清单_FC.md`（FC 失效保护，RELINQUISH 测试前提）、`阶段性报告.md`、`analyze_flight.py`/`diag_monitor.py`。
