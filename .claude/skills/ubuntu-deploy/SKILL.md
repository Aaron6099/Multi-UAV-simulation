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

**步骤 0：清理残留进程（每次启动前必须执行）**
```bash
pkill -f px4; pkill -f gz; pkill -f MicroXRCEAgent; pkill -f ros2
```

**步骤 1：拉取最新代码**
```bash
cd ~/ros2_control_mpc_ws
git pull origin main
```

**步骤 2：赋予启动脚本执行权限（首次拉取后执行一次即可）**
```bash
chmod +x src/mpc_control/start_1_px4.sh
chmod +x src/mpc_control/start_2_px4.sh
chmod +x src/mpc_control/start_3_px4.sh
chmod +x src/mpc_control/start_9_px4.sh
```

**步骤 3：清理 acados 缓存（修改了 mpc_node.py 后必须执行）**
```bash
rm -rf /tmp/acados_di_mpc_*
```

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
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation solo1
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
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation pair2
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
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation trio3
```

**cross5 / star5 / grid9（正式编队）**
```bash
# 终端1
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf
# 终端2
START_DELAY=10 bash ~/ros2_multi_offboard_ws/src/flocking_swarm/start_9_px4.sh
# 终端3
MicroXRCEAgent udp4 -p 8888
# 终端4
cd ~/ros2_control_mpc_ws && source install/setup.bash
ros2 launch mpc_control swarm_launch.py formation:=<队形> [leader_mode:=<模式>] [leader_speed:=1.5] [leader_radius:=10.0]
# 终端5
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation <队形>
```

---

正常启动标志：
- Gazebo 中出现对应数量的无人机模型
- MicroXRCEAgent 显示 `[CREATE  CLIENT]` session 建立
- 各 mpc_node 输出 `acados OCP ready`
- 约 2 s 后输出 `OFFBOARD + ARMED confirmed`
- diag_monitor 显示所有机 ARM=ARMED, NAV=OFFBOARD
