# PX4 多机仿真完整指南

> 综合整理自 PX4 官方文档、GitHub 仓库、本地代码及参考实现
> 整理日期：2026-05-26

---

## 目录

1. [官方文档索引](#1-官方文档索引)
2. [仿真器对比](#2-仿真器对比)
3. [Gazebo Garden 多机仿真（推荐）](#3-gazebo-garden-多机仿真推荐)
4. [Gazebo Classic 多机仿真](#4-gazebo-classic-多机仿真)
5. [SIH / JMAVSim 轻量多机](#5-sih--jmavsim-轻量多机)
6. [Topic 命名规范（关键）](#6-topic-命名规范关键)
7. [uXRCE-DDS 配置](#7-uxrcedds-配置)
8. [QoS 配置（踩坑重点）](#8-qos-配置踩坑重点)
9. [Offboard 控制模式](#9-offboard-控制模式)
10. [Arming 解锁流程](#10-arming-解锁流程)
11. [PX4 v1.14 多机变化](#11-px4-v114-多机变化)
12. [真机部署方案](#12-真机部署方案)
13. [本地代码汇总](#13-本地代码汇总)
14. [已知问题与解决方案](#14-已知问题与解决方案)
15. [参考链接](#15-参考链接)

---

## 1. 官方文档索引

| 文档 | URL |
|------|-----|
| 多机仿真总览 | https://docs.px4.io/main/en/simulation/multi-vehicle-simulation.html |
| Gazebo Garden 多机 | https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation.html |
| Gazebo Classic 多机 | https://docs.px4.io/main/en/sim_gazebo_classic/multi_vehicle_simulation.html |
| ROS 2 多机 | https://docs.px4.io/main/en/ros2/multi_vehicle.html |
| uXRCE-DDS 配置 | https://docs.px4.io/main/en/middleware/uxrce_dds.html |
| Offboard 控制示例 | https://docs.px4.io/main/en/ros2/offboard_control.html |
| Offboard 飞行模式 | https://docs.px4.io/main/en/flight_modes/offboard.html |
| ROS 2 用户指南 | https://docs.px4.io/main/en/ros2/user_guide.html |
| SIH 多机仿真 | https://docs.px4.io/main/en/sim_sih/index.html |
| JMAVSim 多机 | https://docs.px4.io/main/en/sim_jmavsim/multi_vehicle.html |
| FlightGear 多机 | https://docs.px4.io/main/en/sim_flightgear/multi_vehicle.html |
| 伴飞电脑配置 | https://docs.px4.io/main/en/companion_computer/pixhawk_companion.html |

---

## 2. 仿真器对比

| 仿真器 | 重量 | 支持机型 | 最大数量 | 适用场景 |
|--------|------|---------|---------|---------|
| **Gazebo Garden/Harmonic** | 中 | 全部 | 无硬限制 | **推荐**：PX4 v1.14+ 默认 |
| **Gazebo Classic** | 中 | 全部 | 254 | 旧版 PX4，MAVROS 方案 |
| **FlightGear** | 重 | 全部 | 少量 | 高精度仿真 |
| **JMAVSim** | 轻 | 仅四旋翼 | 少量 | 快速原型验证 |
| **SIH** | 最轻 | 全部6种 | 无硬限制 | 无头仿真，零依赖 |

---

## 3. Gazebo Garden 多机仿真（推荐）

### 3.1 启动方式

每个 PX4 实例用唯一的 `-i <instance>` 编号启动：

```bash
# 第1个实例（启动 Gazebo server）
PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 \
  ./build/px4_sitl_default/bin/px4 -i 0

# 后续实例（连接已有 Gazebo server）
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 \
  PX4_GZ_MODEL_POSE="3,0,0,0,0,0" \
  ./build/px4_sitl_default/bin/px4 -i 1

PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL=x500 \
  PX4_GZ_MODEL_POSE="-3,0,0,0,0,0" \
  ./build/px4_sitl_default/bin/px4 -i 2
```

### 3.2 环境变量

| 变量 | 说明 | 示例 |
|------|------|------|
| `PX4_GZ_MODEL` | 要生成的模型名 | `x500`, `gz_rc_cessna` |
| `PX4_GZ_MODEL_POSE` | 生成位姿 (Gazebo ENU) | `"x,y,z,roll,pitch,yaw"` |
| `PX4_GZ_MODEL_NAME` | 绑定已有模型 | 已存在的模型名 |
| `PX4_GZ_STANDALONE` | 不自动启动 Gazebo | `1` |
| `PX4_SYS_AUTOSTART` | 机架 ID | `4001` (x500) |
| `PX4_GZ_WORLD` | 世界文件 | `default` |

### 3.3 机架 ID 对照

| ID | 机型 |
|----|------|
| 4001 | gz_x500（四旋翼，推荐） |
| 4003 | gz_rc_cessna（固定翼） |
| 4009 | gz_standard_vtol（垂直起降） |
| 4010 | gz_r1_rover（地面车） |

### 3.4 3 机示例脚本

```bash
#!/bin/bash
# start_3_px4.sh
PX4_DIR=~/PX4-Autopilot-1.14
cd "$PX4_DIR"

POSES=("0,0,0,0,0,0" "3,0,0,0,0,0" "-3,0,0,0,0,0")

for i in 0 1 2; do
    gnome-terminal --tab --title="px4_$i" -- bash -c "
        export PX4_GZ_STANDALONE=1
        export PX4_SYS_AUTOSTART=4001
        export PX4_GZ_MODEL=x500
        export PX4_GZ_MODEL_POSE='${POSES[$i]}'
        cd '$PX4_DIR'
        ./build/px4_sitl_default/bin/px4 -i $i
        exec bash
    "
    sleep 3
done
echo "3 PX4 instances launched."
```

### 3.5 9 机示例脚本（3x3 方阵）

```bash
#!/bin/bash
# start_9_px4.sh
PX4_DIR=~/PX4-Autopilot-1.14
cd "$PX4_DIR"

declare -a POSES=(
    "0,0,0,0,0,0"     # 0: 中心
    "3,0,0,0,0,0"     # 1: 东
    "-3,0,0,0,0,0"    # 2: 西
    "0,3,0,0,0,0"     # 3: 北
    "0,-3,0,0,0,0"    # 4: 南
    "3,3,0,0,0,0"     # 5: 东北
    "3,-3,0,0,0,0"    # 6: 东南
    "-3,3,0,0,0,0"    # 7: 西北
    "-3,-3,0,0,0,0"   # 8: 西南
)

START_DELAY=3  # 每个实例间隔秒数

for i in $(seq 0 8); do
    gnome-terminal --tab --title="px4_$i" -- bash -c "
        export PX4_GZ_STANDALONE=1
        export PX4_SYS_AUTOSTART=4001
        export PX4_GZ_MODEL=x500
        export PX4_GZ_MODEL_POSE='${POSES[$i]}'
        cd '$PX4_DIR'
        ./build/px4_sitl_default/bin/px4 -i $i
        exec bash
    "
    [ $i -lt 8 ] && sleep "$START_DELAY"
done
echo "All 9 PX4 instances launched."
```

### 3.6 完整仿真启动流程（4 个终端）

```
终端1: gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf
终端2: START_DELAY=3 bash start_9_px4.sh
终端3: MicroXRCEAgent udp4 -p 8888
终端4: ros2 launch mpc_control swarm_launch.py formation:=cross5
```

### 3.7 清理命令

```bash
pkill -f px4; pkill -f gz; pkill -f MicroXRCEAgent; pkill -f ros2
```

---

## 4. Gazebo Classic 多机仿真

### 4.1 官方启动脚本

```bash
# 基本用法
Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n 3 -m iris

# 混合机型
Tools/simulation/gazebo-classic/sitl_multiple_run.sh -s "iris:3,plane:2"

# 指定世界
Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n 5 -m iris -w warehouse
```

### 4.2 参数说明

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `-n <num>` | 无人机数量 | 3 |
| `-m <model>` | 机型 | iris |
| `-w <world>` | 世界 | empty |
| `-s <script>` | 混合脚本 | 无 |
| `-p <pose_map>` | 自定义位置 | 自动 |

### 4.3 支持的机型

- `iris` — 四旋翼（默认）
- `plane` — 固定翼
- `standard_vtol` — 垂直起降
- `rover` — 地面车
- `r1_rover` — 地面车（另一种）
- `typhoon_h480` — 六旋翼

### 4.4 端口分配

| 实例 | TCP | UDP | MAVLink ID |
|------|-----|-----|------------|
| 0 | 4560 | 14560 | 1 |
| 1 | 4561 | 14561 | 2 |
| N | 4560+N | 14560+N | 1+N |

### 4.5 MAVROS 多机（ROS 1）

```bash
roslaunch px4 multi_uav_mavros_sitl.launch
```

Launch 文件位于 `~/PX4-Autopilot-1.14/launch/multi_uav_mavros_sitl.launch`，默认 3 架，注释中说明可扩展到 10 架。

---

## 5. SIH / JMAVSim 轻量多机

### 5.1 SIH（最轻量，无头）

```bash
./Tools/simulation/sitl_multiple_run.sh 3 sihsim_quadx px4_sitl_sih
```

- 无外部依赖，无图形界面
- 支持所有 6 种机型
- 适合算法验证和 CI/CD

### 5.2 JMAVSim（轻量，仅四旋翼）

```bash
./Tools/sitl_multiple_run.sh 2
# 然后对每个实例：
./Tools/simulation/jmavsim/jmavsim_run.sh -p $((4560+i)) -l
```

---

## 6. Topic 命名规范（关键）

### 6.1 原理

PX4 启动脚本 `init.d-posix/rcS` 自动设置：

```bash
param set MAV_SYS_ID $((px4_instance+1))
param set UXRCE_DDS_KEY $((px4_instance+1))

if [ $px4_instance -ne 0 ]; then
    uxrce_dds_ns="-n px4_$px4_instance"
fi
```

### 6.2 命名规则

| px4_instance | UXRCE_DDS_KEY | ROS 2 Topic 前缀 |
|-------------|---------------|------------------|
| 0 | 1 | `/fmu/in/...`, `/fmu/out/...`（无前缀） |
| 1 | 2 | `/px4_1/fmu/in/...`, `/px4_1/fmu/out/...` |
| 2 | 3 | `/px4_2/fmu/in/...`, `/px4_2/fmu/out/...` |
| N | N+1 | `/px4_N/fmu/in/...`, `/px4_N/fmu/out/...` |

### 6.3 自定义命名空间

```bash
# 环境变量覆盖
PX4_UXRCE_DDS_NS=uav_1 ./build/px4_sitl_default/bin/px4 -i 1
# 结果: /uav_1/fmu/in/..., /uav_1/fmu/out/...

# 运行时覆盖
uxrce_dds_client start -n uav_1
```

### 6.4 PX4 v1.17+ 新增

`UXRCE_DDS_NS_IDX` 参数可自动生成 `/uav_0`, `/uav_1` 等命名空间。

### 6.5 ROS 2 代码中的 Topic 映射

```python
def topic_for_drone(drone_id, suffix):
    if drone_id == 0:
        return f'/fmu/{suffix}'
    return f'/px4_{drone_id}/fmu/{suffix}'
```

### 6.6 VehicleCommand 目标系统

```python
msg.target_system = drone_id + 1  # MAV_SYS_ID = px4_instance + 1
```

`VehicleCommand` 只接受 `target_system = 0`（广播）或 `= MAV_SYS_ID`。

---

## 7. uXRCE-DDS 配置

### 7.1 关键参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `UXRCE_DDS_CFG` | 端口选择 | TELEM2 |
| `UXRCE_DDS_PRT` | Agent UDP 端口 | 8888 |
| `UXRCE_DDS_AG_IP` | Agent IP | 2130706433 (127.0.0.1) |
| `UXRCE_DDS_KEY` | 会话密钥（多机必须唯一） | 自动 |
| `UXRCE_DDS_DOM_ID` | DDS 域 ID | 0 |
| `UXRCE_DDS_SYNCT` | 时间同步 | 启用 |

### 7.2 环境变量覆盖

| 环境变量 | 对应参数 |
|---------|---------|
| `PX4_UXRCE_DDS_NS` | 命名空间 |
| `ROS_DOMAIN_ID` | `UXRCE_DDS_DOM_ID` |
| `PX4_UXRCE_DDS_PORT` | `UXRCE_DDS_PRT` |

### 7.3 Agent 安装（ROS 2 Humble 用 v2.4.2）

```bash
git clone -b v2.4.2 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make && sudo make install && sudo ldconfig /usr/local/lib/
```

### 7.4 仿真启动

```bash
# 单个 Agent 处理所有实例
MicroXRCEAgent udp4 -p 8888
```

### 7.5 DDS Topic 列表

**PX4 发布（out）：**
- `vehicle_attitude` — 四元数姿态
- `vehicle_local_position` — 本地 NED 位置
- `vehicle_global_position` — 全局位置
- `vehicle_odometry` — 里程计
- `vehicle_status` — 解锁/导航状态
- `sensor_combined` — 传感器数据
- `vehicle_control_mode` — 控制模式
- `failsafe_flags` — 故障标志
- `timesync_status` — 时间同步

**PX4 订阅（in）：**
- `offboard_control_mode` — Offboard 控制模式
- `trajectory_setpoint` — 轨迹设定点
- `vehicle_command` — 车辆命令（解锁、模式切换）
- `vehicle_attitude_setpoint` — 姿态设定点
- `vehicle_rates_setpoint` — 角速率设定点
- `vehicle_mocap_odometry` — 动捕里程计
- `vehicle_visual_odometry` — 视觉里程计

---

## 8. QoS 配置（踩坑重点）

### 8.1 PX4 端 QoS

**PX4 发布者（DataWriter，"out" topics）：**
```cpp
uxrQoS_t qos = {
    .durability = UXR_DURABILITY_TRANSIENT_LOCAL,
    .reliability = UXR_RELIABILITY_BEST_EFFORT,
    .history = UXR_HISTORY_KEEP_LAST,
    .depth = 0,
};
```

**PX4 订阅者（DataReader，"in" topics）：**
```cpp
uxrQoS_t qos = {
    .durability = UXR_DURABILITY_VOLATILE,      // 注意：与发布者不同！
    .reliability = UXR_RELIABILITY_BEST_EFFORT,
    .history = UXR_HISTORY_KEEP_LAST,
    .depth = queue_depth,
};
```

### 8.2 ROS 2 端 QoS（推荐配置）

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# 订阅 PX4 "out" topics（接收 PX4 数据）
qos_sub = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,  # 匹配 PX4 DataWriter
)

# 发布到 PX4 "in" topics（发送控制命令）
qos_pub = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.VOLATILE,  # 匹配 PX4 DataReader！
)
```

### 8.3 常见 QoS 错误

| 错误 | 后果 | 修复 |
|------|------|------|
| 发布者用 `TRANSIENT_LOCAL`，PX4 期望 `VOLATILE` | 消息可能丢失，多机时更严重 | 改为 `VOLATILE` |
| 订阅者用 `VOLATILE`，PX4 发布者用 `TRANSIENT_LOCAL` | 可以工作但丢失历史消息 | 改为 `TRANSIENT_LOCAL` |
| `reliability` 不匹配 | 完全收不到消息 | 统一用 `BEST_EFFORT` |

### 8.4 Leader 节点 QoS（非 PX4 topic）

```python
# leader_node 使用 RELIABLE（非 PX4 topic，用默认 QoS）
leader_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
```

---

## 9. Offboard 控制模式

### 9.1 官方模式

1. **流式发送 `OffboardControlMode`**（>2 Hz，作为心跳）
2. 发送 10 个设定点后（~1s），发送模式切换 + 解锁命令
3. 继续流式发送 `OffboardControlMode` + `TrajectorySetpoint`

### 9.2 OffboardControlMode 标志

```python
msg = OffboardControlMode()
msg.position = True      # 位置控制
msg.velocity = True      # 速度控制
msg.acceleration = False # 加速度控制
msg.attitude = False     # 姿态控制
msg.body_rate = False    # 机体角速率控制
```

**注意：** 可以同时启用 `position=True` 和 `velocity=True`，PX4 会同时处理。

### 9.3 TrajectorySetpoint 消息

```python
msg = TrajectorySetpoint()
msg.position = [x, y, z]        # NED 坐标，NaN 表示忽略
msg.velocity = [vx, vy, vz]     # NED 速度，NaN 表示忽略
msg.acceleration = [ax, ay, az] # NED 加速度，NaN 表示忽略
msg.yaw = yaw                   # 偏航角（弧度）
msg.yawspeed = float('nan')     # 偏航角速率
```

### 9.4 混合控制模式（位置 Z + 速度 XY）

```python
# OffboardControlMode
msg.position = True    # 启用位置控制（用于 Z 轴）
msg.velocity = True    # 启用速度控制（用于 XY 轴）

# TrajectorySetpoint
msg.position = [float('nan'), float('nan'), target_z]  # 只设 Z 位置
msg.velocity = [vx, vy, float('nan')]                   # 只设 XY 速度
```

### 9.5 坐标系转换

- **Gazebo**：ENU（东-北-天）
- **PX4**：NED（北-东-地）
- 转换：`NED_x = ENU_y`, `NED_y = ENU_x`, `NED_z = -ENU_z`

---

## 10. Arming 解锁流程

### 10.1 方式一：独立 Arming 节点

```python
# arming_node.py
# 状态机: WAITING -> SET_MODE -> ARM -> DONE

# 1. 等待控制器开始发送设定点（>2Hz）
time.sleep(setup_seconds)

# 2. 发送 OFFBOARD 模式命令
msg = VehicleCommand()
msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
msg.param1 = 1.0
msg.param2 = 6.0  # OFFBOARD mode
msg.target_system = drone_id + 1
pub.publish(msg)

# 3. 发送解锁命令
msg = VehicleCommand()
msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
msg.param1 = 1.0  # 1=arm, 0=disarm
msg.target_system = drone_id + 1
pub.publish(msg)
```

### 10.2 方式二：嵌入式 Arming（在控制节点中）

```python
# 在 control_loop 中
def _arm_and_engage_offboard(self):
    if self._arm_offboard_confirmed:
        return
    # 先确保在 OFFBOARD 模式
    if self._nav_state == 14 and self._arming_state != 2:
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
    if self._nav_state == 14 and self._arming_state == 2:
        self._arm_offboard_confirmed = True
```

### 10.3 关键状态码

| nav_state | 含义 |
|-----------|------|
| 0 | MANUAL |
| 1 | ALTCTL |
| 2 | POSCTL |
| 3 | AUTO_MISSION |
| 4 | AUTO_LOITER |
| 14 | **OFFBOARD** |
| 17 | AUTO_RTL |

| arming_state | 含义 |
|-------------|------|
| 1 | DISARMED |
| 2 | **ARMED** |

### 10.4 启动时序

```
时间轴:
|-- 流式发送零速度设定点 (>2Hz) --|-- 发送 OFFBOARD 命令 --|-- 发送 ARM 命令 --|-- 控制 --|
|         ~1s                     |      ~0.5s            |     ~0.5s        |  持续    |
```

**PX4 要求：** 在切换到 OFFBOARD 模式之前，必须已经以 >2Hz 的频率流式发送设定点。

### 10.5 COM_OF_LOSS_T 参数

PX4 的 Offboard 模式丢失超时（默认 0.5s）。如果超过此时间未收到设定点，PX4 自动退出 OFFBOARD 模式。

```bash
# 在 PX4 shell 中增加超时
param set COM_OF_LOSS_T 2.0
```

---

## 11. PX4 v1.14 多机变化

### 11.1 关键变化

- **uXRCE-DDS 替代 Fast-RTPS Bridge**：v1.13 用 Fast-RTPS，v1.14 用 uXRCE-DDS
- `px4_msgs` 包需要匹配 PX4 版本分支
- 迁移指南：https://docs.px4.io/main/en/middleware/uxrce_dds.html#fast-rtps-to-uxrce-dds-migration-guidelines

### 11.2 启动脚本自动配置

```bash
# init.d-posix/rcS 中的多实例设置
param set MAV_SYS_ID $((px4_instance+1))
param set UXRCE_DDS_KEY $((px4_instance+1))

# 命名空间逻辑
uxrce_dds_ns=""
if [ "$px4_instance" -ne "0" ]; then
    uxrce_dds_ns="-n px4_$px4_instance"
fi
if [ "${PX4_UXRCE_DDS_NS+x}" ]; then
    if [ -n "$PX4_UXRCE_DDS_NS" ]; then
        uxrce_dds_ns="-n $PX4_UXRCE_DDS_NS"
    else
        uxrce_dds_ns=""
    fi
fi

# 端口和域
uxrce_dds_port=8888
if [ -n "$PX4_UXRCE_DDS_PORT" ]; then
    uxrce_dds_port="$PX4_UXRCE_DDS_PORT"
fi

uxrce_dds_client start -t udp -p $uxrce_dds_port $uxrce_dds_ns
```

### 11.3 MAVLink 端口分配

| 实例 | 本地 UDP | 远程 UDP | GCS |
|------|---------|---------|-----|
| 0 | 14580 | 14540 | 18570 |
| 1 | 14581 | 14541 | 18571 |
| N | 14580+N | 14540+N | 18570+N |

> 注意：超过 10 个实例时，远程端口上限为 14549。

---

## 12. 真机部署方案

### 12.1 架构

```
[地面站 PC]
    │
    ├── MicroXRCEAgent (UDP)
    │
    ├── [无人机 1] Pixhawk 6C + 树莓派 4B
    │   └── MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
    │
    ├── [无人机 2] Pixhawk 6C + 树莓派 4B
    │   └── MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
    │
    └── [无人机 N] ...
```

### 12.2 每架无人机配置

1. **PX4 参数设置：**
   - `MAV_SYS_ID` = 唯一 ID（1, 2, 3, ...）
   - `UXRCE_DDS_KEY` = 唯一非零值
   - `UXRCE_DDS_CFG` = TELEM2
   - `UXRCE_DDS_PRT` = 8888

2. **伴飞电脑（树莓派 4B）：**
   ```bash
   # 串口连接 Pixhawk
   sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
   ```

3. **或 UDP 连接（WiFi/以太网）：**
   ```bash
   # 每架无人机有不同的 UXRCE_DDS_KEY
   MicroXRCEAgent udp4 -p 8888
   ```

### 12.3 单地面站多机方案

- 每架无人机有独立的伴飞电脑和 Agent
- 或所有无人机通过 WiFi 连接到同一个 Agent（需要不同的 `UXRCE_DDS_KEY`）
- ROS 2 节点运行在地面站 PC 上

### 12.4 真机注意事项

- 使用 `px4_msgs` 匹配的版本分支
- 串口波特率：921600（默认）
- WiFi 延迟可能导致 OFFBOARD 模式丢失，增加 `COM_OF_LOSS_T`
- 真机需要 RC 遥控器作为安全开关

---

## 13. 本地代码汇总

### 13.1 MPC 编队控制器

| 项目 | 路径 | 说明 |
|------|------|------|
| MPC 控制包 | `~/ros2_control_mpc_ws/src/mpc_control/` | acados 双积分器 MPC |
| 启动脚本 | `swarm_launch.py` | 支持 cross5/star5/grid9 |
| 控制节点 | `mpc_node.py` | 每架无人机一个 MPC 求解器 |
| 虚拟领队 | `leader_node.py` | hover/circle/line 模式 |
| PX4 启动 | `start_9_px4.sh` | 9 机 3x3 方阵 |

**MPC 参数：**
- `N=20`, `dt=0.05`（1s 预测时域）
- `q_pos=4.0`, `q_vel=1.0`, `r_acc=0.1`
- `d_safe=1.5m`, `w_collision=200.0`, `w_formation=0.5`
- 控制频率：50Hz

### 13.2 Flocking 编队控制器

| 项目 | 路径 | 说明 |
|------|------|------|
| Flocking 包 | `~/ros2_multi_offboard_ws/src/flocking_swarm/` | 分布式一致性算法 |
| 控制节点 | `flock_controller_node.py` | LMI 增益 |
| 启动脚本 | `swarm_launch.py` | 9 机 3x3 方阵 |

**LMI 增益：** `Kp=0.6983`, `Kv=2.1929`, `alpha=0.5`

### 13.3 C++ 双机编队

| 项目 | 路径 | 说明 |
|------|------|------|
| C++ 控制 | `~/ros2_multi_offboard_ws/src/multi_offboard_control/` | 2 机圆周飞行 |
| 控制节点 | `multi_offboard_control.cpp` | 100ms 循环 |

### 13.4 PX4_Swarm_Controller（Ecole Centrale de Nantes）

| 项目 | 路径 | 说明 |
|------|------|------|
| Swarm 控制器 | `~/PX4_Swarm_Controller/` | C++ 加权拓扑 |
| 配置 | `config/swarm_config.json` | 3 架 iris |
| 控制配置 | `config/control_config.json` | PID 增益 |
| 轨迹 | `config/Trajectories/` | 圆形、上下 |

### 13.5 参考实现

| 项目 | 路径 | 说明 |
|------|------|------|
| px4-mpc | `~/ref/px4-mpc/` | acados MPC 单机 |
| px4-offboard | `~/ref/px4-offboard/` | ETH Zurich Python 示例 |
| reswarm_dmpc | `~/ref/reswarm_dmpc/` | NASA Astrobee 3 机分布式 MPC |

---

## 14. 已知问题与解决方案

### 14.1 OFFBOARD 模式丢失

**症状：** 所有无人机同时退出 OFFBOARD 模式（nav_state 从 14 变为 2）

**原因：**
1. QoS 不匹配（`TRANSIENT_LOCAL` vs `VOLATILE`）
2. `COM_OF_LOSS_T` 超时（默认 0.5s）
3. MicroXRCEAgent 过载

**解决方案：**
```python
# 1. 修复 QoS
qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,  # 匹配 PX4 DataReader
)

# 2. 增加 OffboardControlMode 标志
msg.position = True   # 不要只设 velocity=True
msg.velocity = True

# 3. 在 PX4 中增加超时
# param set COM_OF_LOSS_T 2.0
```

### 14.2 EKF 重置

**症状：** `xy_reset_counter` / `z_reset_counter` 增加，位置跳变

**解决方案：**
```python
# 动态跟踪 world_birth 偏移
if msg.xy_reset_counter > self._prev_xy_reset[drone_idx]:
    self.world_birth[drone_idx, 0] -= float(msg.delta_xy[0])
    self.world_birth[drone_idx, 1] -= float(msg.delta_xy[1])
    self._prev_xy_reset[drone_idx] = msg.xy_reset_counter
```

### 14.3 解锁失败

**症状：** 部分无人机无法解锁（nav=14, arm=1 但不变成 arm=2）

**原因：** EKF 未收敛，需要更多时间

**解决方案：**
- 增加 `WAIT_EKF=1` 在 start_9_px4.sh 中
- 增加启动间隔（`START_DELAY`）
- 增加 `startup_zero_vel_frames`

### 14.4 邻居超时

**症状：** `NEIGHBOR X TIMEOUT` 警告

**解决方案：**
```python
# 增加超时时间
neighbour_timeout = 2.0  # 从 0.5 增加到 2.0
```

### 14.5 编队振荡

**症状：** 无人机编队飞行时振荡或飞散

**解决方案：**
```python
# 降低编队权重
w_formation = 0.5  # 从 3.0 降低到 0.5

# 降低最大速度
max_speed = 3.0  # 从 5.0 降低到 3.0
```

---

## 15. 参考链接

### 官方仓库

| 仓库 | URL |
|------|-----|
| PX4-Autopilot | https://github.com/PX4/PX4-Autopilot |
| px4_ros_com | https://github.com/PX4/px4_ros_com |
| px4_msgs | https://github.com/PX4/px4_msgs |
| Micro-XRCE-DDS-Agent | https://github.com/eProsima/Micro-XRCE-DDS-Agent |

### 官方示例

| 示例 | URL |
|------|-----|
| C++ Offboard 控制 | https://github.com/PX4/px4_ros_com/blob/main/src/examples/offboard/offboard_control.cpp |
| Python Offboard 控制 | https://github.com/Jaeyoung-Lim/px4-offboard |

### 社区项目

| 项目 | 说明 |
|------|------|
| Jaeyoung-Lim/px4-offboard | ETH Zurich Python Offboard 示例（官方文档引用） |
| PX4_Swarm_Controller | Ecole Centrale de Nantes C++ 编队控制器 |
| reswarm_dmpc | NASA Astrobee 分布式 MPC |

---

## 附录 A：快速复制命令

### 仿真启动（4 终端）

```bash
# 终端1 - Gazebo
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf

# 终端2 - PX4 实例
START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_5_px4.sh

# 终端3 - DDS Agent
MicroXRCEAgent udp4 -p 8888

# 终端4 - 控制器
cd ~/ros2_control_mpc_ws && colcon build --packages-select mpc_control && source install/setup.bash
ros2 launch mpc_control swarm_launch.py formation:=cross5
```

### 清理

```bash
pkill -f px4; pkill -f gz; pkill -f MicroXRCEAgent; pkill -f ros2
```

### 检查 Topic

```bash
ros2 topic list
ros2 topic echo /fmu/out/vehicle_status
ros2 topic echo /px4_1/fmu/out/vehicle_local_position
```

---

## 附录 B：PX4 x500 模型参数

| 参数 | 值 |
|------|-----|
| 质量 | 2.0 kg |
| 旋翼数 | 4 |
| 电机常数 | 8.54858e-06 |
| 最大转速 | 1000 rad/s |
| 悬停油门 | 0.60 |
| 传感器 | 气压计(50Hz), IMU(250Hz), GPS, 磁力计 |
| 机架 ID | 4001 |

---

*文档完成于 2026-05-26*
