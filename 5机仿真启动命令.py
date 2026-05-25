完整操作命令（按顺序在四个终端中执行）

步骤0：清理残留进程（每次启动前执行一次）

pkill -f px4
pkill -f gz
pkill -f MicroXRCEAgent
pkill -f ros2

终端1：启动 Gazebo
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf

终端2：启动 PX4 实例（9架，适用所有队形）
START_DELAY=10 bash ~/ros2_multi_offboard_ws/src/flocking_swarm/start_9_px4.sh

终端3：启动 MicroXRCEAgent
MicroXRCEAgent udp4 -p 8888

终端4：编译并启动编队控制器
cd ~/ros2_control_mpc_ws
colcon build --packages-select mpc_control
source install/setup.bash

========== 选择队形（三选一）==========

【5机十字编队】
ros2 launch mpc_control swarm_launch.py formation:=cross5

【5机星型编队（正五边形）】
ros2 launch mpc_control swarm_launch.py formation:=star5

【9机3×3方阵】
ros2 launch mpc_control swarm_launch.py formation:=grid9

========================================

二、队形说明

5机十字（cross5）：
        3(北)
   2(西) 0(中) 1(东)
        4(南)
  间距3米，drone 0~4 参与编队

5机星型（star5）：
  0 (北, 0,3)
  1 (东北, 2.85,0.93)
  2 (东南, 1.76,-2.43)
  3 (西南, -1.76,-2.43)
  4 (西北, -2.85,0.93)
  正五边形，半径3米，drone 0~4 参与编队

9机3×3方阵（grid9）：
  7(西北) 3(北) 5(东北)
  2(西)   0(中) 1(东)
  8(西南) 4(南) 6(东南)
  间距3米，drone 0~8 全部参与编队

注意：5机模式下 drone 5~8 会出现在 Gazebo 中但停在地面不参与编队，属正常现象。

三、切换队形方法（不需要重启终端1/2/3）
Ctrl+C 停止终端4，然后重新运行所选队形的 launch 命令。
切换不同队形（如 cross5 → star5）不需要重新 colcon build。
首次运行或修改代码后才需要重新编译。

四、领队运动模式（修改 swarm_launch.py 中 leader_node 的参数）

5机十字（cross5）
# 悬停
ros2 launch mpc_control swarm_launch.py formation:=cross5

# 匀速圆周 半径10m 速度1.5m/s
ros2 launch mpc_control swarm_launch.py formation:=cross5 leader_mode:=circle leader_speed:=1.5 leader_radius:=10.0

# 直线 1m/s
ros2 launch mpc_control swarm_launch.py formation:=cross5 leader_mode:=line leader_speed:=1.0

5机星型（star5）
# 悬停
ros2 launch mpc_control swarm_launch.py formation:=star5

# 匀速圆周 半径10m 速度1.5m/s
ros2 launch mpc_control swarm_launch.py formation:=star5 leader_mode:=circle leader_speed:=1.5 leader_radius:=10.0

# 直线 1m/s
ros2 launch mpc_control swarm_launch.py formation:=star5 leader_mode:=line leader_speed:=1.0

9机3×3方阵（grid9）
# 悬停
ros2 launch mpc_control swarm_launch.py formation:=grid9
 
# 匀速圆周 半径10m 速度1.5m/s
ros2 launch mpc_control swarm_launch.py formation:=grid9 leader_mode:=circle leader_speed:=1.5 leader_radius:=10.0
 
# 直线 1m/s
ros2 launch mpc_control swarm_launch.py formation:=grid9 leader_mode:=line leader_speed:=1.0

# 换队形+换运动模式随意组合
ros2 launch mpc_control swarm_launch.py formation:=star5 leader_mode:=circle leader_radius:=8.0

悬停（默认）：
  'mode': 'hover'

匀速圆周：
  'mode': 'circle'
  'speed': 1.0      # 飞行速度 m/s
  'radius': 10.0    # 圆半径 m

直线飞行：
  'mode': 'line'
  'speed': 1.0      # 飞行速度 m/s

五、复现检查清单
Gazebo 正常启动且无报错
start_9_px4.sh 执行后看到9个无人机模型出现在 Gazebo 中
MicroXRCEAgent 显示 [INFO] [xrce] session ... 无错误
终端4中每个 mpc_node 输出 acados OCP ready 和 mpc_controller drone x ready
约1秒后各飞机输出 sent ARM + OFFBOARD commands
Gazebo 中无人机起飞并形成对应编队