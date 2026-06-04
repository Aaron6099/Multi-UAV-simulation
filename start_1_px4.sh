#!/bin/bash
# start_1_px4.sh — solo1 单机诊断
#
# 用法:
#   终端1: gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf
#   终端2: bash start_1_px4.sh
#   终端3: MicroXRCEAgent udp4 -p 8888
#   终端4: ros2 launch mpc_control swarm_launch.py formation:=solo1
#
# 出生位置 (Gazebo ENU: x=East, y=North):
#   drone 0: 中心 (0, 0) → NED (0, 0)
#
# Env: PX4_DIR — PX4-Autopilot 路径, 默认 ~/PX4-Autopilot-1.14

set -e

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot-1.14}"

if [ ! -d "$PX4_DIR" ]; then
    echo "ERROR: PX4_DIR=$PX4_DIR 不存在"
    echo "请设置: export PX4_DIR=~/PX4-Autopilot"
    exit 1
fi

export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"

cd "$PX4_DIR"

POSE="0,0,0,0,0,0"
echo "启动 drone 0 | ENU pose: $POSE"

if command -v gnome-terminal &> /dev/null; then
    gnome-terminal --tab --title="px4_0_solo" -- bash -c "
        export GZ_SIM_RESOURCE_PATH='$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds'
        export PX4_GZ_STANDALONE=1
        export PX4_SYS_AUTOSTART=4001
        export PX4_GZ_MODEL=x500
        export PX4_GZ_MODEL_POSE='$POSE'
        cd '$PX4_DIR'
        ./build/px4_sitl_default/bin/px4 -i 0
        exec bash
    "
else
    LOG_DIR="$HOME/px4_logs"
    mkdir -p "$LOG_DIR"
    (
        export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"
        export PX4_GZ_STANDALONE=1
        export PX4_SYS_AUTOSTART=4001
        export PX4_GZ_MODEL=x500
        export PX4_GZ_MODEL_POSE="$POSE"
        cd "$PX4_DIR"
        ./build/px4_sitl_default/bin/px4 -i 0 > "$LOG_DIR/px4_0.log" 2>&1
    ) &
    echo "  後台运行，日志: $LOG_DIR/px4_0.log (PID $!)"
fi

echo "=== solo1: 1 架 PX4 已启动 ==="
