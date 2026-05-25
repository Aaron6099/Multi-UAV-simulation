#!/usr/bin/env python3
"""
启动 MPC 编队，支持三种队形：
  cross5  — 5机十字编队
  star5   — 5机五边形星型编队
  grid9   — 9机3×3方阵

用法:
  ros2 launch mpc_control swarm_launch.py formation:=cross5
  ros2 launch mpc_control swarm_launch.py formation:=star5
  ros2 launch mpc_control swarm_launch.py formation:=grid9
"""
import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ── 出生位置（必须与 start_9_px4.sh 的 POSES 完全一致）─────────────────────
# 坐标系：PX4 NED（x=北/North, y=东/East, z=下/Down）
# Gazebo ENU (x=East,y=North) 与 PX4 NED 的换算：NED_x=ENU_y, NED_y=ENU_x
BIRTH_5 = [
     0.0,  0.0, 0.0,   # 0 中心
     0.0,  3.0, 0.0,   # 1 东  (NED: y=East=+3)
     0.0, -3.0, 0.0,   # 2 西  (NED: y=East=-3)
     3.0,  0.0, 0.0,   # 3 北  (NED: x=North=+3)
    -3.0,  0.0, 0.0,   # 4 南  (NED: x=North=-3)
]

BIRTH_9 = BIRTH_5 + [
     3.0,  3.0, 0.0,   # 5 东北 (NED: N=+3, E=+3)
    -3.0,  3.0, 0.0,   # 6 东南 (NED: N=-3, E=+3)
     3.0, -3.0, 0.0,   # 7 西北 (NED: N=+3, E=-3)
    -3.0, -3.0, 0.0,   # 8 西南 (NED: N=-3, E=-3)
]

# ── 编队偏移（相对于虚拟领队的期望位置）────────────────────────────────────
OFFSETS_CROSS5 = BIRTH_5[:]   # 十字：出生即队形

_R = 3.0
OFFSETS_STAR5 = []
for _i in range(5):
    _a = math.radians(90.0 - _i * 72.0)   # 从正北开始，顺时针，72°间隔
    # NED: North=sin(a)*R, East=cos(a)*R（注意与ENU的cos/sin互换）
    OFFSETS_STAR5 += [round(_R * math.sin(_a), 4),
                      round(_R * math.cos(_a), 4), 0.0]
# NED结果：(3,0) (0.927,2.853) (-2.427,1.763) (-2.427,-1.763) (0.927,-2.853)

OFFSETS_GRID9 = BIRTH_9[:]    # 3×3：出生即队形

# ── 每架无人机的邻居列表（用于编队保持和碰撞避免）─────────────────────────
NBR_CROSS5 = [
    [1, 2, 3, 4],   # 0 中心 → 四臂
    [0],            # 1 东
    [0],            # 2 西
    [0],            # 3 北
    [0],            # 4 南
]

NBR_STAR5 = [
    [1, 4],   # 0 ↔ 相邻五边形顶点
    [0, 2],   # 1
    [1, 3],   # 2
    [2, 4],   # 3
    [3, 0],   # 4
]

NBR_GRID9 = [
    [1, 2, 3, 4],   # 0 中心
    [0, 5, 6],      # 1 东
    [0, 7, 8],      # 2 西
    [0, 5, 7],      # 3 北
    [0, 6, 8],      # 4 南
    [1, 3],         # 5 东北
    [1, 4],         # 6 东南
    [2, 3],         # 7 西北
    [2, 4],         # 8 西南
]

FORMATIONS = {
    'cross5': dict(num=5, birth=BIRTH_5, offsets=OFFSETS_CROSS5, nbr=NBR_CROSS5),
    'star5':  dict(num=5, birth=BIRTH_5, offsets=OFFSETS_STAR5,  nbr=NBR_STAR5),
    'grid9':  dict(num=9, birth=BIRTH_9, offsets=OFFSETS_GRID9,  nbr=NBR_GRID9),
}

COMMON = {
    'target_alt':              -5.0,   # NED，负值=向上，-5.0 → 离地5米
    'max_speed':                5.0,
    'max_climb':                2.0,
    'max_accel':                5.0,
    'control_hz':              50.0,
    'neighbour_timeout':        0.5,
    'startup_zero_vel_frames':   50,
    'mpc_horizon':              20,
    'mpc_dt':                 0.05,
    'q_pos':                   4.0,
    'q_vel':                   1.0,
    'r_acc':                   0.1,
    'q_pos_terminal_scale':    2.0,
    'd_safe':                  1.2,
    'w_collision':           200.0,
    'w_formation':             3.0,   # 原0.5太弱，提高编队保持力
    'acados_build_dir': '/tmp/acados_di_mpc',
}


def _make_nodes(context, *args, **kwargs):
    formation = LaunchConfiguration('formation').perform(context)
    if formation not in FORMATIONS:
        raise ValueError(
            f'未知队形 "{formation}"，可选: {list(FORMATIONS.keys())}')
    cfg = FORMATIONS[formation]

    nodes = []

    # 虚拟领队节点
    nodes.append(Node(
        package='mpc_control',
        executable='leader_node',
        name='leader_node',
        output='screen',
        parameters=[{
            'mode':        LaunchConfiguration('leader_mode'),
            'start_x':      0.0,
            'start_y':      0.0,
            'altitude':     COMMON['target_alt'],
            'speed':        LaunchConfiguration('leader_speed'),
            'radius':       LaunchConfiguration('leader_radius'),
            'publish_hz':  50.0,
        }],
    ))

    # 每架无人机的 MPC 控制节点
    for drone_id in range(cfg['num']):
        p = dict(COMMON)
        p['drone_id']               = drone_id
        p['num_drones']             = cfg['num']
        p['birth_positions_flat']   = cfg['birth']
        p['formation_offsets_flat'] = cfg['offsets']
        p['neighbours']             = cfg['nbr'][drone_id]
        nodes.append(Node(
            package='mpc_control',
            executable='mpc_node',
            name=f'mpc_node_{drone_id}',
            namespace=f'px4_{drone_id}',
            output='screen',
            parameters=[p],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'formation',
            default_value='cross5',
            description='cross5 | star5 | grid9',
        ),
        DeclareLaunchArgument(
            'leader_mode',
            default_value='hover',
            description='领队运动模式: hover | circle | line',
        ),
        DeclareLaunchArgument(
            'leader_speed',
            default_value='1.0',
            description='circle/line 模式下的飞行速度 (m/s)',
        ),
        DeclareLaunchArgument(
            'leader_radius',
            default_value='10.0',
            description='circle 模式下的圆半径 (m)',
        ),
        OpaqueFunction(function=_make_nodes),
    ])
