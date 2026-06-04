#!/bin/bash
# start_2_px4.sh — pair2 双机诊断（前后纵列，间距 3 m）
#
# 用法:
#   终端1: gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf
#   终端2: START_DELAY=5 bash start_2_px4.sh
#   终端3: MicroXRCEAgent udp4 -p 8888
#   终端4: ros2 launch mpc_control swarm_launch.py formation:=pair2
#
# 出生位置 (Gazebo ENU: x=East, y=North):
#   drone 0: (ENU 0,0)   → NED (  0,  0) 中心
#   drone 1: (ENU 0,-3)  → NED ( -3,  0) 南 3 m
#
# NED↔ENU: ENU_x = NED_y(East), ENU_y = NED_x(North)
#
# Env: PX4_DIR, START_DELAY (默认 5 s)

set -e

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot-1.14}"
START_DELAY="${START_DELAY:-5}"

if [ ! -d "$PX4_DIR" ]; then
    echo "ERROR: PX4_DIR=$PX4_DIR 不存在"
    exit 1
fi

export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds"

cd "$PX4_DIR"

# Gazebo ENU 出生位置  "x_east, y_north, z_up, roll, pitch, yaw"
declare -a POSES=(
    "0,0,0,0,0,0"    # 0: 中心        NED( 0,   0)
    "0,-3,0,0,0,0"   # 1: 南 3 m      NED(-3,   0)
)

for i in 0 1; do
    POSE="${POSES[$i]}"
    echo "启动 drone $i | ENU pose: $POSE"

    if command -v gnome-terminal &> /dev/null; then
        gnome-terminal --tab --title="px4_${i}_pair2" -- bash -c "
            export GZ_SIM_RESOURCE_PATH='$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds'
            export PX4_GZ_STANDALONE=1
            export PX4_SYS_AUTOSTART=4001
            export PX4_GZ_MODEL=x500
            export PX4_GZ_MODEL_POSE='$POSE'
            cd '$PX4_DIR'
            ./build/px4_sitl_default/bin/px4 -i $i
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
            ./build/px4_sitl_default/bin/px4 -i $i > "$LOG_DIR/px4_$i.log" 2>&1
        ) &
        echo "  後台，日志: $LOG_DIR/px4_$i.log (PID $!)"
    fi

    if [ $i -lt 1 ]; then
        echo "  等待 ${START_DELAY}s ..."
        sleep "$START_DELAY"
    fi
done

echo "=== pair2: 2 架 PX4 已启动 ==="
