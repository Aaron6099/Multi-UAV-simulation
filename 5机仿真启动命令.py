完整操作命令（按顺序在终端中执行）

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
步骤0：每次启动前清理残留进程
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

pkill -f px4; pkill -f gz; pkill -f MicroXRCEAgent; pkill -f ros2

mkdir -p ~/flights   # CSV 记录目录（首次建一次即可，供终端5 的 --log 使用）


━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
更新代码（改完代码后执行）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

cd ~/ros2_control_mpc_ws/src/mpc_control && git pull origin main   # 仓库即 src/mpc_control，git pull 直接更新模块，无需 cp
cd ~/ros2_control_mpc_ws
rm -rf /tmp/acados_di_mpc_*        # 仅在改了 MPC OCP 结构(horizon/状态/邻居数/约束)时才需清缓存
colcon build --packages-select mpc_control
source install/setup.bash
# 注：diag_monitor.py 直接 python3 运行，git pull 即生效，无需 colcon build


━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
【Phase 0】单机诊断 solo1  ← 必须先过这关
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

终端1：
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf

终端2（等Gazebo出现画面后）：
START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_1_px4.sh

终端3：
MicroXRCEAgent udp4 -p 8888

终端4：
cd ~/ros2_control_mpc_ws && source install/setup.bash
ros2 launch mpc_control swarm_launch.py formation:=solo1

终端5（诊断监控 + 记录CSV）：
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation solo1 --log ~/flights/flight_solo1_hover.csv

队形图（NED）：
  0(中心 0,0)


━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
【Phase 1】双机诊断 pair2  ← solo1 稳定后执行
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

终端1：
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf

终端2：
START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_2_px4.sh

终端3：
MicroXRCEAgent udp4 -p 8888

终端4（悬停）：
cd ~/ros2_control_mpc_ws && source install/setup.bash
ros2 launch mpc_control swarm_launch.py formation:=pair2

终端4（圆周 半径10m 速度1.5m/s）：
ros2 launch mpc_control swarm_launch.py formation:=pair2 leader_mode:=circle leader_speed:=1.5 leader_radius:=10.0

终端4（直线 1m/s）：
ros2 launch mpc_control swarm_launch.py formation:=pair2 leader_mode:=line leader_speed:=1.0

终端5（诊断监控 + 记录CSV；<traj> 换成终端4 选的 hover/line/circle）：
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation pair2 --log ~/flights/flight_pair2_<traj>.csv

队形图（NED，间距3m）：
  0(中心  0, 0)
  1(南   -3, 0)


━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
【Phase 2】三机诊断 trio3  ← pair2 稳定后执行
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

终端1：
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf

终端2：
START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_3_px4.sh

终端3：
MicroXRCEAgent udp4 -p 8888

终端4（悬停）：
cd ~/ros2_control_mpc_ws && source install/setup.bash
ros2 launch mpc_control swarm_launch.py formation:=trio3

终端4（圆周 半径10m 速度1.5m/s）：
ros2 launch mpc_control swarm_launch.py formation:=trio3 leader_mode:=circle leader_speed:=1.5 leader_radius:=10.0

终端4（直线 1m/s）：
ros2 launch mpc_control swarm_launch.py formation:=trio3 leader_mode:=line leader_speed:=1.0

终端5（诊断监控 + 记录CSV；<traj> 换成终端4 选的 hover/line/circle）：
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation trio3 --log ~/flights/flight_trio3_<traj>.csv

队形图（NED，等边三角形 外接圆R=3m 边长≈5.196m）：
       0(北 +3,0)
      / \
     /   \
  2(西南) 1(东南)
  (-1.5,-2.598) (-1.5,+2.598)


━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
【Phase 3】5机编队  ← trio3 稳定后执行
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

终端1：
gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf

终端2：
START_DELAY=5 bash ~/ros2_control_mpc_ws/src/mpc_control/start_5_px4.sh

终端3：
MicroXRCEAgent udp4 -p 8888

终端4（5机十字 悬停）：
cd ~/ros2_control_mpc_ws && source install/setup.bash
ros2 launch mpc_control swarm_launch.py formation:=cross5

终端4（5机十字 圆周）：
ros2 launch mpc_control swarm_launch.py formation:=cross5 leader_mode:=circle leader_speed:=1.5 leader_radius:=10.0

终端4（5机十字 直线）：
ros2 launch mpc_control swarm_launch.py formation:=cross5 leader_mode:=line leader_speed:=1.0

终端4（5机星型 悬停）：
ros2 launch mpc_control swarm_launch.py formation:=star5

终端4（5机星型 圆周）：
ros2 launch mpc_control swarm_launch.py formation:=star5 leader_mode:=circle leader_speed:=1.5 leader_radius:=10.0

终端5（诊断监控 + 记录CSV；star5 时把两处 cross5 换成 star5，<traj> 换 hover/line/circle）：
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation cross5 --log ~/flights/flight_cross5_<traj>.csv

队形图 cross5（间距3m）：
        3(北)
   2(西) 0(中) 1(东)
        4(南)

队形图 star5（正五边形 R=3m）：
  0(北  0,3)
  1(东北 2.85,0.93)
  2(东南 1.76,-2.43)
  3(西南 -1.76,-2.43)
  4(西北 -2.85,0.93)

注意：5机用 start_5_px4.sh 正好启动5架、全部参与编队；9机方阵见下一节（用 start_9_px4.sh）。


━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
【Phase 3】9机3×3方阵  ← cross5/star5 稳定后执行
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

终端4（悬停）：
ros2 launch mpc_control swarm_launch.py formation:=grid9

终端4（圆周）：
ros2 launch mpc_control swarm_launch.py formation:=grid9 leader_mode:=circle leader_speed:=1.5 leader_radius:=10.0

终端4（直线）：
ros2 launch mpc_control swarm_launch.py formation:=grid9 leader_mode:=line leader_speed:=1.0

终端5（诊断监控 + 记录CSV；<traj> 换成终端4 选的 hover/line/circle）：
python3 ~/ros2_control_mpc_ws/src/mpc_control/diag_monitor.py --formation grid9 --log ~/flights/flight_grid9_<traj>.csv

队形图 grid9（间距3m）：
  7(西北) 3(北) 5(东北)
  2(西)   0(中) 1(东)
  8(西南) 4(南) 6(东南)


━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
切换队形方法（不需要重启终端1/2/3）
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Ctrl+C 停止终端4 → 重新运行所选队形的 launch 命令
切换队形不需要重新 colcon build
修改代码后才需要重新编译


━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
正常启动检查清单
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

□ Gazebo 正常启动无报错
□ start_N_px4.sh 执行后 Gazebo 中出现对应数量无人机模型
□ MicroXRCEAgent 显示 [CREATE  CLIENT] session 建立
□ 各 mpc_node 输出 acados OCP ready
□ 约2秒后输出 OFFBOARD + ARMED confirmed
□ diag_monitor 显示所有机 ARM=ARMED  NAV=OFFBOARD
□ diag_monitor 显示 Min spacing > 1.8m
□ diag_monitor 显示 MPC fallback_count = 0
