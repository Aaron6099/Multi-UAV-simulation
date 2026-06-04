#!/bin/bash
# start_3_px4.sh — trio3 三机诊断（等边三角形，外接圆半径 3 m，边长 ≈ 5.196 m）
#
# 用法:
#   终端1: gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf
#   终端2: START_DELAY=5 bash start_3_px4.sh
#   终端3: MicroXRCEAgent udp4 -p 8888
#   终端4: ros2 launch mpc_control swarm_launch.py formation:=trio3
#
# 出生位置 (Gazebo ENU: x=East, y=North):
#   drone 0: (ENU  0,   3)    → NED (+3,      0   ) 北顶
#   drone 1: (ENU  2.598,-1.5)→ NED (-1.5,   +2.598) 东南
#   drone 2: (ENU -2.598,-1.5)→ NED (-1.5,   -2.598) 西南
#
# NED↔ENU: ENU_x = NED_y(East), ENU_y = NED_x(North)
# 最小间距 = 3√3 ≈ 5.196 m >> d_safe=1.5 m，安全余量充足
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
    "0,3,0,0,0,0"          # 0: 北顶        NED(+3,     0    )
    "2.598,-1.5,0,0,0,0"   # 1: 东南        NED(-1.5,  +2.598)
    "-2.598,-1.5,0,0,0,0"  # 2: 西南        NED(-1.5,  -2.598)
)

for i in 0 1 2; do
    POSE="${POSES[$i]}"
    echo "启动 drone $i | ENU pose: $POSE"

    if command -v gnome-terminal &> /dev/null; then
        gnome-terminal --tab --title="px4_${i}_trio3" -- bash -c "
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

    if [ $i -lt 2 ]; then
        echo "  等待 ${START_DELAY}s ..."
        sleep "$START_DELAY"
    fi
done

echo "=== trio3: 3 架 PX4 已启动 ==="
