#!/bin/bash
# start_5_px4.sh — cross5 / star5 五机编队
#
# 用法:
#   终端1: gz sim -r ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf
#   终端2: START_DELAY=5 bash start_5_px4.sh
#   终端3: MicroXRCEAgent udp4 -p 8888
#   终端4: ros2 launch mpc_control swarm_launch.py formation:=cross5
#       或: ros2 launch mpc_control swarm_launch.py formation:=star5
#
# 出生位置 (Gazebo ENU: x=East, y=North):
#   drone 0: (ENU  0,  0) → NED ( 0,  0) 中心
#   drone 1: (ENU  3,  0) → NED ( 0, +3) 东
#   drone 2: (ENU -3,  0) → NED ( 0, -3) 西
#   drone 3: (ENU  0,  3) → NED (+3,  0) 北
#   drone 4: (ENU  0, -3) → NED (-3,  0) 南
#
# 注: cross5 和 star5 共用同一套出生位置（都是十字型）
#     star5 的队形偏移在 swarm_launch.py 中用 OFFSETS_STAR5 重定义
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
    "0,0,0,0,0,0"    # 0: 中心  NED( 0,  0)
    "3,0,0,0,0,0"    # 1: 东    NED( 0, +3)
    "-3,0,0,0,0,0"   # 2: 西    NED( 0, -3)
    "0,3,0,0,0,0"    # 3: 北    NED(+3,  0)
    "0,-3,0,0,0,0"   # 4: 南    NED(-3,  0)
)

for i in 0 1 2 3 4; do
    POSE="${POSES[$i]}"
    echo "启动 drone $i | ENU pose: $POSE"

    if command -v gnome-terminal &> /dev/null; then
        gnome-terminal --tab --title="px4_${i}_5uav" -- bash -c "
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

    if [ $i -lt 4 ]; then
        echo "  等待 ${START_DELAY}s ..."
        sleep "$START_DELAY"
    fi
done

echo ""
echo "=== cross5/star5: 5 架 PX4 实例已启动 ==="
echo "编队布局 (NED):"
echo "  drone 0: (  0,  0) 中心"
echo "  drone 1: (  0, +3) 东"
echo "  drone 2: (  0, -3) 西"
echo "  drone 3: ( +3,  0) 北"
echo "  drone 4: ( -3,  0) 南"
echo ""
echo "下一步:"
echo "  终端3: MicroXRCEAgent udp4 -p 8888"
echo "  终端4: cd ~/ros2_control_mpc_ws && source install/setup.bash"
echo "          ros2 launch mpc_control swarm_launch.py formation:=cross5"
echo "       或: ros2 launch mpc_control swarm_launch.py formation:=star5"
