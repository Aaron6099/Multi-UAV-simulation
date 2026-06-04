#!/bin/bash
# start_9_px4.sh
# 启动 9 个 PX4 SITL 实例,组成 3×3 方阵编队
#
# 用法:
#   1. 先在另一个终端启动 Gazebo:
#        cd ~/PX4-Autopilot-1.14
#        gz sim -r -s Tools/simulation/gz/worlds/default.sdf
#   2. 等 Gazebo 起来后,本脚本启动 9 个 PX4 实例(每个开新的 gnome-terminal tab)
#   3. 然后在另开的终端运行:
#        MicroXRCEAgent udp4 -p 8888
#        ros2 launch <你的包> swarm_launch.py
#
# 坐标系:PX4_GZ_MODEL_POSE = "x,y,z,roll,pitch,yaw" (Gazebo ENU)
#   x = East (东为正), y = North (北为正), z = Up
#
# 编号约定:
#   drone 0: 中心 (0, 0)
#   drone 1: 东   (3, 0)
#   drone 2: 西   (-3, 0)
#   drone 3: 北   (0, 3)
#   drone 4: 南   (0, -3)
#   drone 5: 东北 (3, 3)
#   drone 6: 东南 (3, -3)
#   drone 7: 西北 (-3, 3)
#   drone 8: 西南 (-3, -3)
#
# Env vars:
#   PX4_DIR     - PX4-Autopilot 源码目录, 默认 ~/PX4-Autopilot-1.14
#   START_DELAY - 启动间隔(秒), 默认 3

set -e

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot-1.14}"
START_DELAY="${START_DELAY:-3}"

if [ ! -d "$PX4_DIR" ]; then
    echo "ERROR: PX4_DIR=$PX4_DIR does not exist."
    echo "Set PX4_DIR env var, e.g.:  PX4_DIR=~/PX4-Autopilot bash start_9_px4.sh"
    exit 1
fi

export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"

cd "$PX4_DIR"

# 9 架飞机的 6 参数位姿 (x,y,z,roll,pitch,yaw)
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

for i in $(seq 0 8); do
    POSE="${POSES[$i]}"
    echo "Starting drone $i with PX4_GZ_MODEL_POSE=$POSE ..."

    if command -v gnome-terminal &> /dev/null; then
        gnome-terminal --tab --title="px4_$i" -- bash -c "
            export GZ_SIM_RESOURCE_PATH='$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds'
            export PX4_GZ_STANDALONE=1
            export PX4_SYS_AUTOSTART=4001
            export PX4_GZ_MODEL=x500
            export PX4_GZ_MODEL_POSE='$POSE'
            cd '$PX4_DIR'
            ./build/px4_sitl_default/bin/px4 -i $i;
            exec bash
        "
    else
        # Fallback: 后台 + 日志文件
        LOG_DIR="$HOME/px4_logs"
        mkdir -p "$LOG_DIR"
        (
            export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"
            export PX4_GZ_STANDALONE=1
            export PX4_SYS_AUTOSTART=4001
            export PX4_GZ_MODEL=x500
            export PX4_GZ_MODEL_POSE="$POSE"
            cd "$PX4_DIR"
            ./build/px4_sitl_default/bin/px4 -i $i > "$LOG_DIR/px4_$i.log" 2>&1
        ) &
        echo "  -> background, log: $LOG_DIR/px4_$i.log (PID $!)"
    fi

    if [ $i -lt 8 ]; then
        echo "  waiting ${START_DELAY}s before next instance..."
        sleep "$START_DELAY"
    fi
done

echo "=== grid9: 9 架 PX4 已启动 ==="
