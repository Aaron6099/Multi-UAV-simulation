# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

PX4 v1.14 多机编队仿真，5/9 架无人机，ROS2 + acados MPC 控制。
代码在 **Windows 桌面编辑**，通过 GitHub 同步到 **Ubuntu 主机**进行仿真。

## 坐标系（极易出错）

| 系统 | 坐标轴 | 说明 |
|------|--------|------|
| PX4 NED | x=北, y=东, z=下 | 所有 setpoint、position 话题均用此系 |
| Gazebo ENU | x=东, y=北, z=上 | 换算：NED_x=ENU_y, NED_y=ENU_x |

内部统一用 **世界系 NED**：`ds.pos = local_pos + world_birth[drone_id]`
发给 PX4 前须转回本地系：`pos_local = pos_world - world_birth[drone_id]`

## 控制架构

- **控制模式**：`OffboardControlMode.position=True`（位置闭环）
- **Setpoint**：`TrajectorySetpoint.position`（非 NaN）+ `.velocity`（前馈）
- **PX4 行为**：position 非 NaN 时，velocity 作为内环前馈项（官方文档确认）
- **MPC 输出**：`x_pred[1, 0:3]` → position setpoint；`x_pred[1, 3:6]` → velocity feedforward
- **降级策略**：任何异常（解失败 / 偏差 >5m / NaN）→ `_hover_setpoint_world()` 悬停

## 关键参数位置

所有运行参数集中在 `swarm_launch.py` 的 `COMMON` 字典，修改此处即可，无需改 `mpc_node.py`。

重要参数：
- `max_speed`: 3.0 m/s（位置控制模式下的速度前馈限幅）
- `neighbour_timeout`: 2.0 s（邻居通信超时容忍）
- `target_alt`: -5.0（NED，负值=向上，离地 5 米）
- `d_safe`: 1.5 m（碰撞软约束距离）

## 话题命名规则

- drone 0：`/fmu/in/...`、`/fmu/out/...`
- drone 1-8：`/px4_{id}/fmu/in/...`、`/px4_{id}/fmu/out/...`
- 领队：`/leader/state`（Float32MultiArray，格式 `[t, x, y, z, vx, vy, vz, yaw]`）
- MPC 预测轨迹：`/mpc/predicted_trajectory`（drone 0）或 `/px4_{id}/mpc/predicted_trajectory`

## 队形配置

| 队形 | 无人机数 | 拓扑 | 间距 |
|------|----------|------|------|
| cross5 | 5 | 十字，中心连四臂 | 3 m |
| star5 | 5 | 正五边形，环形邻居 | R=3 m |
| grid9 | 9 | 3×3 方阵 | 3 m |

队形偏移定义在 `swarm_launch.py`：`OFFSETS_*`；邻居列表：`NBR_*`。

## Ubuntu 端构建与启动顺序

```bash
# 1. 清理残留
pkill -f px4; pkill -f gz; pkill -f MicroXRCEAgent; pkill -f ros2

# 2. 终端1：Gazebo
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf

# 3. 终端2：PX4（等 Gazebo 启动后）
START_DELAY=10 bash ~/ros2_multi_offboard_ws/src/flocking_swarm/start_9_px4.sh

# 4. 终端3：DDS 桥
MicroXRCEAgent udp4 -p 8888

# 5. 终端4：编译并启动（从 GitHub 拉取新代码后执行）
cd ~/ros2_control_mpc_ws
git pull  # 拉取最新代码
cp ~/Desktop/或对应路径/mpc_node.py src/mpc_control/  # 若未直接 clone
colcon build --packages-select mpc_control
source install/setup.bash
ros2 launch mpc_control swarm_launch.py formation:=cross5
```

## acados 编译缓存

每架无人机首次运行会编译 C 代码到 `/tmp/acados_di_mpc_v{drone_id}_m{neighbours_count}/`。
**修改 MPC 结构（horizon N、状态维度、邻居数）后必须删除该目录**，否则使用旧缓存导致崩溃：
```bash
rm -rf /tmp/acados_di_mpc_*
```

## 已知陷阱

- **EKF 重置**：`world_birth` 数组动态补偿 EKF 跳变；直接用 `ds.pos` 已是世界系，勿再加偏移
- **5 机模式**：drone 5-8 在 Gazebo 中可见但停在地面，属正常现象
- **MPC 求解器**：`SQP_RTI` 单次迭代，`nlp_solver_max_iter=30` 不影响此模式；修改结构需重新编译 acados
- **启动顺序**：Gazebo 必须先于 PX4 实例启动，否则无人机模型无法加载

## 开发工作流

```
Windows 桌面编辑代码
    → /commit-push 提交推送到 GitHub
    → Ubuntu: git pull → colcon build → 仿真验证
```
