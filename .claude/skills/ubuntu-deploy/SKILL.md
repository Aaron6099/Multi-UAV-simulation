---
name: ubuntu-deploy
description: 输出在 Ubuntu 仿真主机上拉取最新代码、编译并启动仿真的完整命令序列。在 Windows 端改完代码 push 后，用此 skill 生成 Ubuntu 端需要运行的命令。
disable-model-invocation: false
---

用户运行 /ubuntu-deploy 时，询问：
1. 需要哪种队形？(solo1 / pair2 / trio3 / cross5 / star5 / grid9)
2. 领队模式？(hover / circle / line)
3. 是否需要清理 acados 缓存？（修改了 mpc_node.py 或 MPC 结构时选是，默认选是）

然后输出以下命令块，用户可直接复制到 Ubuntu 终端执行：

---

**步骤 0：清理残留进程（每次启动前必须执行；逐个明确进程名，pkill -9 -f 不会误伤 claude）**
```bash
for p in px4 'gz sim' gzserver MicroXRCEAgent mpc_node leader_node swarm_launch 'ros2 launch'; do pkill -9 -f "$p"; done
ros2 daemon stop 2>/dev/null; pkill -9 -f 'ros2-daemon'
# 必做验证：下面这条应「零输出」才算清干净（gz sim server 会忽略信号、最易残留，需按 PID kill -9）
ps aux | grep -E "px4|gz sim|gzserver|MicroXRCE|mpc_node|leader_node|swarm_launch|ros2-daemon" | grep -v grep
```

**步骤 1：拉取最新代码**
```bash
cd ~/ros2_control_mpc_ws
git pull origin main
mkdir -p ~/flights          # CSV 记录目录（首次建一次即可，供终端5 的 --log 使用）
```

**步骤 2：赋予启动脚本执行权限（首次拉取后执行一次即可）**
```bash
chmod +x src/mpc_control/start_*_px4.sh   # 覆盖 1/2/3/5/9 全部脚本
```

**步骤 3：清理 acados 缓存（仅当改了 MPC「OCP 结构」：horizon N / 状态维 / 约束 / 邻居数）**
```bash
rm -rf /tmp/acados_di_mpc_*
```
> ⚠️ 纯 Python / 参数改动（标定、leader、就绪门控、记录器等）**不必清缓存**，清了只是白等几分钟重编译。只有动了 acados OCP 结构才需清。

**步骤 4：编译**
```bash
cd ~/ros2_control_mpc_ws
colcon build --packages-select mpc_control
source install/setup.bash
```

---

根据队形选择对应的启动命令：

**solo1（单机诊断，Phase 0）**
```bash
# 终端1
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf
# 终端2（等 Gazebo 就绪后）
START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_1_px4.sh
# 终端3
MicroXRCEAgent udp4 -p 8888
# 终端4
cd ~/ros2_control_mpc_ws && source install/setup.bash
ros2 launch mpc_control swarm_launch.py formation:=solo1
# 终端5（诊断监控）
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation solo1 --log ~/flights/flight_solo1_<traj>.csv
```

**pair2（双机，Phase 1）**
```bash
# 终端1
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf
# 终端2
START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_2_px4.sh
# 终端3
MicroXRCEAgent udp4 -p 8888
# 终端4
cd ~/ros2_control_mpc_ws && source install/setup.bash
ros2 launch mpc_control swarm_launch.py formation:=pair2 [leader_mode:=hover]
# 终端5
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation pair2 --log ~/flights/flight_pair2_<traj>.csv
```

**trio3（三机，Phase 2）**
```bash
# 终端1
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf
# 终端2
START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_3_px4.sh
# 终端3
MicroXRCEAgent udp4 -p 8888
# 终端4
cd ~/ros2_control_mpc_ws && source install/setup.bash
ros2 launch mpc_control swarm_launch.py formation:=trio3 [leader_mode:=hover]
# 终端5
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation trio3 --log ~/flights/flight_trio3_<traj>.csv
```

**cross5 / star5（5机，Phase 3）**
```bash
# 终端1
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf
# 终端2（5机务必用 start_5_px4.sh；误用 start_9 会让 drone5-8 停在地面）
START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_5_px4.sh
# 终端3
MicroXRCEAgent udp4 -p 8888
# 终端4
cd ~/ros2_control_mpc_ws && source install/setup.bash
ros2 launch mpc_control swarm_launch.py formation:=<cross5|star5> [leader_mode:=<模式>] [leader_speed:=1.5] [leader_radius:=10.0]
# 终端5
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation <cross5|star5> --log ~/flights/flight_<cross5|star5>_<traj>.csv
```

**grid9（9机，Phase 3）**
```bash
# 终端1
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf
# 终端2
START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_9_px4.sh
# 终端3
MicroXRCEAgent udp4 -p 8888
# 终端4
cd ~/ros2_control_mpc_ws && source install/setup.bash
ros2 launch mpc_control swarm_launch.py formation:=grid9 [leader_mode:=<模式>] [leader_speed:=1.5] [leader_radius:=10.0]
# 终端5
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation grid9 --log ~/flights/flight_grid9_<traj>.csv
```

---

**就绪门控（af59b66）**：leader 不再死等固定 `start_delay`，而是**等所有机进入编队（pos_err<0.5m 保持 2s）才自动开始运动**，日志打 `formation ready — starting`；90s 超时兜底打红字。想退回旧固定延时：launch 加 `ready_gate_enable:=false`。

**飞行记录（3e1b3e9）**：上面终端5 已带 `--log`，每秒写一行 CSV 到 `~/flights/`（`<traj>` 换成所选 hover/line/circle）。跑完用 analyze_flight 出体检报告：
```bash
python3 ~/ros2_control_mpc_ws/src/mpc_control/analyze_flight.py ~/flights/flight_<队形>_<traj>.csv [--plot]
```
> 完整逐场景「控制器+记录」命令见 `report/CORE_run_commands.md`（CORE S1–S8）与 `report/RUN_PLAN_仿真运行清单.md`。

---

正常启动标志：
- Gazebo 中出现对应数量的无人机模型
- MicroXRCEAgent 显示 `[CREATE  CLIENT]` session 建立
- 各 mpc_node 输出 `acados OCP ready`
- 约 2 s 后输出 `OFFBOARD + ARMED confirmed`
- diag_monitor 显示所有机 ARM=ARMED, NAV=OFFBOARD
