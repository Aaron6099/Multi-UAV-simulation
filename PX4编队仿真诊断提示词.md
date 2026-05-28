# PX4 编队仿真诊断提示词
> 目标：从单机基准验证开始，逐步扩展到稳定的 5机/9机 阵列飞行，最终上真机

---

## 当前进度看板

| Phase | 阶段 | 状态 |
|-------|------|------|
| Phase 0 | solo1 单机基准 | 🔄 进行中（T2-T3 偏航/MPC设定点调试中）|
| Phase 1 | pair2 双机 | ⏳ 待开始 |
| Phase 2 | trio3 三机 | ⏳ 待开始 |
| Phase 3 | cross5 / star5 / grid9 | ⏳ 待开始 |

> 每次完成一个 Gate 后在此处更新状态标记：✅ 已通过 / 🔄 进行中 / ⏳ 待开始 / ❌ 失败需返工

---

## 项目上下文

| 项目 | 说明 |
|------|------|
| 仿真环境 | PX4 v1.14 + ROS2 + acados MPC + Gazebo |
| 开发流程 | Windows 桌面编辑 → GitHub → Ubuntu 主机仿真 |
| 控制模式 | `OffboardControlMode.position=True`（位置闭环 + 速度前馈） |
| 坐标系 | PX4 NED（x=北, y=东, z=下）；Gazebo ENU（x=东, y=北, z=上） |
| 最终目标 | 仿真稳定后上真机（Pixhawk + 树莓派 4B），室内外均需飞行 |

> **MPC 设定点说明**：MPC 当前输出 `x_pred[pred_k, 0:3]` 作为 PX4 位置设定点。  
> `pred_k=1` 时预测位置 ≈ 当前位置，PX4 几乎不产生速度指令，导致飞行迟钝。  
> 建议优先验证 `pred_k=3`（0.15s 前瞻），中期改为输出速度设定点（`TrajectorySetpoint.velocity`）。

### 文件路径（Ubuntu）

```
~/ros2_control_mpc_ws/src/mpc_control/
  ├── mpc_node.py          # 每架无人机的 acados MPC 控制节点
  ├── leader_node.py       # 虚拟领队（hover / circle / line），含 yaw_mode 参数
  ├── swarm_launch.py      # 队形配置与启动（solo1/pair2/trio3/cross5/star5/grid9）
  ├── diag_monitor.py      # 实时诊断监控（需 ros2 run 或直接 python3）
  ├── start_1_px4.sh       # 单机 PX4 启动脚本
  ├── start_2_px4.sh       # 双机 PX4 启动脚本
  └── start_3_px4.sh       # 三机 PX4 启动脚本
```

### 关键参数（swarm_launch.py COMMON 字典）

```python
target_alt              = -5.0    # NED，负值=向上，离地 5 m
max_speed               = 3.0     # m/s
max_climb               = 1.5     # m/s
d_safe                  = 1.5     # m，碰撞软约束距离
w_collision             = 200.0   # 碰撞惩罚权重
w_formation             = 0.5     # 编队保持权重
neighbour_timeout       = 2.0     # s
startup_zero_vel_frames = 100     # 帧，2 s EKF 收敛等待
control_hz              = 50.0    # Hz
mpc_horizon N           = 20      # 步
mpc_dt                  = 0.05    # s，预测时域 = 1 s
pred_k                  = 3       # MPC 前瞻步数（建议值，对应 0.15s 前瞻）
```

### 偏航角控制参数（leader_node.py）

```python
yaw_mode = 'fixed'    # 悬停/直线默认值：锁定初始朝向
                      # 可选：'fixed'（固定）/ 'center'（朝向圆心）/ 'tangent'（跟随飞行方向）
```

**运行时切换命令（无需重启节点）：**

```bash
ros2 param set /leader_node yaw_mode fixed    # 固定朝向（编队保持推荐）
ros2 param set /leader_node yaw_mode center   # 朝向圆心（观测/摄影场景）
ros2 param set /leader_node yaw_mode tangent  # 跟随飞行方向（展示场景）
```

> 多机时：所有僚机 yaw = leader_yaw，保持相对朝向一致，不独立计算各自 yaw。

### 各运动模式偏航角要求汇总

| 运动模式 | yaw 要求 | yaw_rate 要求 | 备注 |
|----------|----------|---------------|------|
| 悬停（hover）| 锁定初始值，漂移 < 5° | < 3°/s | EKF 偏航估计基准 |
| 圆周（circle）| 由 yaw_mode 决定 | 平滑，< 45°/s | 出发前先完成对准 |
| 直线（line）| 对准飞行方向后锁定 | 出发前一次性对准 | 禁止倒飞，方向反转先重新对准 |

### 话题命名规则

```python
# drone 0:    /fmu/in/...  /fmu/out/...
# drone 1-N:  /px4_N/fmu/in/...  /px4_N/fmu/out/...
# 领队:        /leader/state  (Float64MultiArray: [t,x,y,z,vx,vy,vz,yaw])
# MPC 诊断:   /mpc/health  或  /px4_N/mpc/health
#             (Float64MultiArray: [drone_id, status, solve_ms, fallback_cnt, hover_active, pos_err])
```

### 已修复的关键 Bug（勿回退）

1. **QoS 不匹配**：`pub_offboard_mode` / `pub_setpoint` / `pub_vehicle_cmd` 已改为 `VOLATILE`（匹配 PX4 DataReader），原 `TRANSIENT_LOCAL` 会导致多机 ARM 失败和 OFFBOARD 丢失
2. **ARM 重试频率**：已从每 100 帧改为每 50 帧（1 s 重试一次）
3. **EKF 收敛时间**：`startup_zero_vel_frames` 已从 50 改为 100
4. **消息类型不匹配**：`leader_node.py` 和 `mpc_node.py` 的 leader 话题已从 `Float32MultiArray` 改为 `Float64MultiArray`，精度不足会导致位置漂移
5. **EKF 重置导致 world_birth 错误**：已添加 `_pos_calibrated` 标志，首次位置校准后锁定，避免重复加偏移

---

## 启动顺序（每次仿真必须按此顺序）

```bash
# 1. 清理残留进程
pkill -f px4; pkill -f gz; pkill -f MicroXRCEAgent; pkill -f ros2

# 2. 终端1：启动 Gazebo（先等 10 s 再进行下一步）
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf

# 3. 终端2：启动 PX4 实例（根据阶段选择脚本）
START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_1_px4.sh  # Phase 0
START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_2_px4.sh  # Phase 1
START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_3_px4.sh  # Phase 2

# 4. 终端3：启动 DDS 桥
MicroXRCEAgent udp4 -p 8888

# 5. 终端4：编译并启动控制器（改代码后必须重新 build）
cd ~/ros2_control_mpc_ws
rm -rf /tmp/acados_di_mpc_*   # 修改 MPC 结构时必须清除缓存
colcon build --packages-select mpc_control
source install/setup.bash
ros2 launch mpc_control swarm_launch.py formation:=solo1   # 按阶段替换

# 5b. 另开终端：启动诊断监控
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation solo1
```

---

## 队形定义

### solo1（单机，Phase 0）

```
drone 0: NED(0, 0, 0) — 中心
邻居: 无
```

### pair2（双机前后纵列，Phase 1）

```
drone 0: NED( 0,  0) — 中心
drone 1: NED(-3,  0) — 南 3 m
间距: 3.0 m
邻居: 0↔1 互为邻居
```

### trio3（三机等边三角形，Phase 2）

```
drone 0: NED(+3.000,  0.000) — 北顶
drone 1: NED(-1.500, +2.598) — 东南
drone 2: NED(-1.500, -2.598) — 西南
外接圆半径: 3 m，边长: ≈ 5.196 m >> d_safe=1.5 m
邻居: 全连接三角（每机 2 个邻居）
```

---

## 验收标准

> **规则：每个 Phase 的所有测试项必须全部通过，才能进入下一个 Phase。禁止跳过。**

### Phase 0 — 单机基准（solo1）

> 目的：排除一切单机层面的问题，这是多机安全的最低门槛。

#### G0-T1 启动健康（30 s 内）

```
□ MicroXRCEAgent 日志出现 "[CREATE  CLIENT]"
□ mpc_node 日志出现 "acados OCP ready"
□ mpc_node 日志出现 "first position from drone 0"
□ mpc_node 日志出现 "OFFBOARD + ARMED confirmed"
□ 启动到 ARM 确认 < 15 s
□ 期间无 "MPC solve crashed" 或 "holding position" 日志
```

#### G0-T2 悬停稳定性（120 s）

```
配置: formation=solo1, leader_mode=hover

□ 起飞后 30 s 内到达 z = -5.0 m（NED 本地系）
□ 高度误差 |z - (-5.0)| < 0.15 m，持续 95% 时间满足
□ 水平漂移 < 0.2 m（相对起飞点，使用世界坐标 = 本地坐标 + world_birth 评估）
□ MPC status 1/3/4 出现次数 = 0（允许 status=2 不超过 5 次）
□ MPC 求解时间 < 8 ms（每帧）
□ 无 EKF xy_reset / z_reset 事件（如有请记录）

偏航角验收：
□ yaw 锁定在起飞时记录的 initial_yaw，漂移 < 5°
□ yaw_rate < 3°/s（无持续自旋）
□ 120 s 内累计偏航漂移 < 10°
```

#### G0-T3 圆周运动跟踪（120 s）

```
配置: leader_mode=circle, speed=1.0, radius=8.0

□ 水平跟踪误差 RMS < 0.4 m
□ 高度误差 |z - (-5.0)| < 0.2 m 持续满足
□ 速度前馈未被 clip（vel_ff_xy_norm ≤ max_speed）
□ MPC status 1/3/4 出现次数 = 0
□ 速度无突变（单帧变化 < 0.5 m/s）

偏航角验收（每种 yaw_mode 单独测试）：
□ fixed 模式：yaw 漂移 < 10°，无自旋行为
□ center 模式：yaw 始终指向圆心，误差 < 15°，yaw_rate 平滑无突变
□ tangent 模式：yaw 跟随速度方向，yaw_rate < 45°/s，无跳变
□ 三种模式下运行时切换（ros2 param set）均生效，切换后 5 s 内 yaw 平滑过渡
```

#### G0-T4 直线飞行（60 s）

```
配置: leader_mode=line, speed=1.0

□ 侧向偏差（y 轴）< 0.2 m
□ 纵向跟踪误差 < 0.3 m
□ 高度误差 < 0.15 m
□ MPC 无异常

偏航角验收：
□ 出发前 yaw 先对准飞行方向（误差 < 10°），对准完成后再开始平移
□ 飞行中 yaw 保持对准，漂移 < 10°
□ yaw_rate 飞行中 < 3°/s（对准阶段除外）
```

#### G0-T5 模式切换稳定性

```
操作: line → hover（Ctrl+C 后重 launch hover）

□ 30 s 内稳定到新悬停点，误差 < 0.2 m
□ 切换过程无连续 5 帧以上 hover fallback
□ 高度切换过程波动 < 0.5 m
□ yaw 切换后 10 s 内收敛到 initial_yaw，无持续震荡
```

#### G0-T6 长时稳定性（10 min hover）

```
□ 水平漂移累积 < 0.3 m
□ MPC 求解时间始终 < 10 ms
□ 无 EKF reset 事件（如有请记录时刻和 world_birth 变化量）
□ 无内存/CPU 异常增长
□ yaw 累积漂移 < 15°（10 min 内）
```

**Gate-0 通过条件：T1-T6 全部通过，且连续两次运行 G0-T2 均通过**

---

### Phase 1 — 双机诊断（pair2）

> 目的：验证邻居通信、MPC 碰撞约束、编队保持。**必须先通过 Gate-0。**

#### G1-T1 双机起飞健康检查

```
□ drone 0 和 drone 1 均出现 "OFFBOARD + ARMED confirmed"
□ 两机均在 30 s 内到达 z = -5.0 m
□ 初始机间距 3.0 m，误差 < 0.1 m
□ 两机互相接收到对方的 predicted_trajectory 话题
□ 无 "MPC solve crashed" 日志
□ 两机 ARM 时间差 < 5 s（如超过请记录原因）
```

#### G1-T2 双机悬停编队保持（120 s）

```
□ 机间距始终 > 1.8 m（d_safe=1.5 m + 0.3 m 余量）
□ 队形偏差（实际间距与期望 3.0 m 之差）绝对值 < 0.4 m，95% 时间满足
□ 两机高度差 < 0.2 m
□ 两机 MPC 均无 status 1/3/4
□ 碰撞事件（机间距 < 1.5 m）= 0 次

偏航角验收：
□ 两机 yaw 均锁定初始朝向，相互偏差 < 5°
□ 无自旋行为（yaw_rate < 3°/s）
```

#### G1-T3 邻居超时降级测试

```
操作: G1-T2 稳定后 kill drone 1 的 mpc_node，观察 drone 0 行为

□ drone 0 在 neighbour_timeout=2.0 s 内仍尝试维持队形
□ 超时后 drone 0 切换到 hover 降级，无 MPC 崩溃
□ 恢复 drone 1 后，drone 0 重新接收邻居数据并恢复编队（< 10 s）
```

#### G1-T4 圆周运动双机编队（120 s）

```
配置: leader_mode=circle, speed=1.5, radius=10.0

□ 最小机间距始终 > 1.8 m
□ 队形偏差 RMS < 0.5 m
□ 轨迹跟踪误差 RMS < 0.5 m
□ MPC status 1/3/4 = 0 次
□ 速度突变 < 1.0 m/s per frame

偏航角验收：
□ 两机 yaw 一致（相互偏差 < 10°），均跟随 leader yaw_mode 设定
□ 切换 yaw_mode 时两机同步响应，5 s 内完成过渡
□ yaw_rate < 45°/s（tangent 模式）或 < 10°/s（其他模式）
```

#### G1-T5 直线飞行双机编队（60 s）

```
配置: leader_mode=line, speed=1.0

□ 最小机间距始终 > 1.8 m
□ 纵向间距（NED x 方向）3.0 m，误差 < 0.3 m
□ 横向对齐（NED y 方向偏差）< 0.2 m
□ 两机 yaw 均对准飞行方向，误差 < 10°
```

#### G1-T6 长时编队稳定性（circle 10 min）

```
□ 10 min 内无碰撞事件（间距 < 1.5 m）
□ 队形偏差无持续增大趋势（后 5 min RMS ≤ 前 5 min RMS × 1.5）
□ 两机 MPC 求解时间 < 12 ms
□ EKF reset 次数记录（如有，验证 world_birth 补偿正确）
□ 两机 yaw 累积漂移 < 15°（10 min 内，fixed 模式）
```

**Gate-1 通过条件：T1-T6 全部通过，且连续两次运行 G1-T2 均通过**

---

### Phase 2 — 三机诊断（trio3）

> 目的：验证多邻居 MPC、三机同步降落。**必须先通过 Gate-1。**

#### G2-T1 三机起飞健康检查

```
□ 三机均在 45 s 内完成 ARM + OFFBOARD（三机启动时间差 < 5 s）
□ 每机 desired_distances 正确（drone 0 与 drone 1/2 间距均 ≈ 5.196 m）
□ 三机互相接收到所有邻居的 predicted_trajectory
□ acados 缓存目录正确区分（检查 /tmp/acados_di_mpc_v0_m2、v1_m2、v2_m2 均存在）
```

#### G2-T2 三机悬停编队保持（120 s）

```
□ 最小机间距始终 > 1.8 m
□ 队形偏差 < 0.5 m，95% 时间满足
□ 三机高度差 < 0.2 m
□ 三机 MPC 均无 status 1/3/4
□ 碰撞事件 = 0 次

偏航角验收：
□ 三机 yaw 均锁定初始朝向，最大相互偏差 < 8°
□ 无任意一机出现自旋行为
```

#### G2-T3 MPC 求解负载验证

```
□ 三机 MPC 求解时间均 < 12 ms（trio3 每机有 2 邻居，问题规模更大）
□ 如求解时间 > 15 ms，记录哪架无人机并分析原因
□ 10 min 圆周运动内无 status 1/3/4
```

#### G2-T4 圆周运动三机编队（120 s）

```
配置: leader_mode=circle, speed=1.5, radius=10.0

□ 最小机间距始终 > 1.8 m
□ 队形偏差 RMS < 0.6 m
□ MPC status 1/3/4 = 0 次
□ 三机 yaw 一致，跟随 leader yaw_mode，相互偏差 < 10°
```

#### G2-T5 直线飞行三机编队（60 s）

```
配置: leader_mode=line, speed=1.0

□ 最小机间距始终 > 1.8 m
□ 队形偏差 < 0.5 m
□ 三机 yaw 均对准飞行方向，误差 < 10°
```

#### G2-T6 三机同步降落

```
操作: Ctrl+C 停止 leader_node，等 5 s 后手动发送降落指令

□ leader 信号丢失后，三机在 2 s 内切换到 hover 降级
□ 降落过程最小间距始终 > 1.0 m
□ 三机均正常落地（z ≈ 0.0 m，PX4 切换 LANDED 状态）
□ 降落过程 MPC 无崩溃
```

**Gate-2 通过条件：T1-T6 全部通过，且连续两次运行 G2-T2 均通过**

---

### Phase 3 — 扩展到 5机 / 9机

> **仅在 Gate-0 + Gate-1 + Gate-2 全部通过后执行。**

扩展顺序（逐步增加，每步用 Phase 1/2 同等标准验证）：

```
cross5（5机十字）→ star5（5机星型）→ grid9（9机3×3方阵）
```

每个队形最低门槛：

```
□ 120 s 悬停：最小间距 > 1.8 m，队形偏差 RMS < 0.6 m，yaw 一致性偏差 < 10°
□ 10 min 圆周（speed=1.5, radius=10）：无碰撞，无 MPC 发散
□ 邻居超时降级测试（kill 一台，其他机安全 hover）
□ 三种 yaw_mode 均可运行时切换，无异常
```

---

## 已知陷阱（实现时必须规避）

| 陷阱 | 说明 | 处理方式 |
|------|------|---------|
| acados 缓存 | 修改 MPC horizon/邻居数/状态维度后缓存失效 | `rm -rf /tmp/acados_di_mpc_*` |
| EKF 重置补偿 | `world_birth` 动态追踪，`ds.pos = local_pos + world_birth` | 勿重复加偏移；`_pos_calibrated` 标志已锁定首次校准 |
| 启动顺序 | Gazebo 必须先于 PX4 实例启动 | Gazebo 就绪后等 10 s |
| drone 0 话题 | 无命名空间前缀（`/fmu/...`），drone 1+ 用 `/px4_N/fmu/...` | 检查 topic_for_drone() |
| QoS 不匹配 | 发布到 PX4 "in" 话题必须用 VOLATILE | 已修复，勿改回 TRANSIENT_LOCAL |
| desired_distances | 从 formation_offsets 的 XY 距离自动计算 | 新增队形时确认 OFFSETS 的 XY 距离正确 |
| solo1 邻居列表 | NBR_SOLO1 = [[0]]，被 mpc_node 自过滤为空 | max_neighbours = max(1, 0) = 1，OCP 结构不变 |
| 5机模式 | drone 5-8 停在地面可见，属正常现象 | 忽略 |
| 圆周运动自旋 | yaw 未锁定时跟随速度切线方向，绕圆一圈 yaw 转 360° | 使用 `yaw_mode=fixed` 或显式设置 yaw_mode 参数 |
| MPC 设定点过近 | `pred_k=1` 时预测位置 ≈ 当前位置，PX4 几乎不动 | 使用 `pred_k=3` 或改为速度设定点输出 |
| leader 话题类型 | 必须用 `Float64MultiArray`，`Float32MultiArray` 精度不足 | 已修复，mpc_node 和 leader_node 均已更新 |

---

## 扩展性约束（代码架构红线）

```
新增队形 → 只在 swarm_launch.py 添加 BIRTH/OFFSETS/NBR，不改 mpc_node.py
每机邻居数 ≤ 4（稀疏化），支持更大阵列不增加 MPC 状态维度
acados 编译目录按 drone_id 和 neighbours_count 区分，不同规模互不干扰
yaw_mode 参数只在 leader_node.py 中计算，僚机直接继承 leader yaw，不独立计算
```

---

## 真机过渡安全检查清单

> **仿真所有 Phase 全部通过后才执行此检查，现阶段仅留存备用。**

```
□ d_safe     真机改为 2.0 m（仿真 1.5 m + 0.5 m 真机余量）
□ max_speed  真机降为 1.5 m/s（仿真 3.0 m/s）
□ max_accel  真机降为 2.0 m/s²（仿真 4.0 m/s²）
□ target_alt 真机首次测试 -3.0 m（仿真 -5.0 m）
□ startup_zero_vel_frames 真机改为 150（3 s）
□ neighbour_timeout 真机改为 1.0 s（真机无线链路延迟高）
□ 每架真机独立运行 solo1 等效测试通过，再飞多机
□ 真机需要 RC 遥控器作为安全开关（COM_OF_LOSS_T 建议 2.0 s）
□ MicroXRCEAgent 改为串口模式（serial --dev /dev/ttyUSB0 -b 921600）
□ 确认每架真机 MAV_SYS_ID 和 UXRCE_DDS_KEY 唯一且非零
```

### 室内真机附加检查（动捕系统）

```
□ PX4 EKF2 参数：关 GPS（EKF2_AID_MASK），开外部视觉（EV）
□ EKF2_HGT_MODE 改为视觉高度，EKF2_EV_DELAY 设为实测延迟（通常 30-50 ms）
□ 动捕刚体坐标系与 PX4 ENU 对齐（一次性旋转矩阵校准）
□ VRPN/ROS2 桥接节点正常发布 /mavros/vision_pose/pose
□ 室内单机 solo1 悬停精度验证后再飞多机
□ 动捕覆盖区域确认 > 编队最大外包圆直径 + 2 m 安全余量
```
