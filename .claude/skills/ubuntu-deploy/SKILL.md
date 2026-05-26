---
name: ubuntu-deploy
description: 输出在 Ubuntu 仿真主机上拉取最新代码、编译并启动仿真的完整命令序列。在 Windows 端改完代码 push 后，用此 skill 生成 Ubuntu 端需要运行的命令。
disable-model-invocation: false
---

用户运行 /ubuntu-deploy 时，询问：
1. 需要哪种队形？(cross5 / star5 / grid9)
2. 领队模式？(hover / circle / line)
3. 是否需要清理 acados 缓存？（修改了 MPC 结构时选是）

然后输出以下命令块，用户可直接复制到 Ubuntu 终端执行：

---

**步骤 0：清理残留进程（每次启动前）**
```bash
pkill -f px4; pkill -f gz; pkill -f MicroXRCEAgent; pkill -f ros2
```

**[若需要清理 acados 缓存]**
```bash
rm -rf /tmp/acados_di_mpc_*
```

**步骤 1：拉取最新代码**
```bash
cd ~/ros2_control_mpc_ws
git pull origin main
```

**步骤 2：编译**
```bash
colcon build --packages-select mpc_control
source install/setup.bash
```

**终端1：启动 Gazebo**
```bash
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf
```

**终端2：启动 PX4（等 Gazebo 完全加载后）**
```bash
START_DELAY=10 bash ~/ros2_multi_offboard_ws/src/flocking_swarm/start_9_px4.sh
```

**终端3：DDS 桥**
```bash
MicroXRCEAgent udp4 -p 8888
```

**终端4：启动编队控制器**
```bash
cd ~/ros2_control_mpc_ws && source install/setup.bash
ros2 launch mpc_control swarm_launch.py formation:=<队形> [leader_mode:=<模式>] [leader_speed:=1.5] [leader_radius:=10.0]
```

---

同时提示用户观察以下正常启动标志：
- Gazebo 中出现 9 架无人机模型
- MicroXRCEAgent 显示 session 建立
- 各 mpc_node 输出 `acados OCP ready`
- 约 1 秒后输出 `sent ARM + OFFBOARD commands`
- 无人机起飞并形成编队
