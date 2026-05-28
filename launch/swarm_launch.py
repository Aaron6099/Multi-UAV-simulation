#!/usr/bin/env python3
"""
启动 MPC 编队，支持以下队形：
  solo1   — 1机单机诊断（Phase 0 基准验证）
  pair2   — 2机前后纵列（Phase 1 双机诊断）
  trio3   — 3机等边三角形（Phase 2 三机诊断）
  cross5  — 5机十字编队
  star5   — 5机五边形星型编队
  grid9   — 9机3×3方阵

用法:
  ros2 launch mpc_control swarm_launch.py formation:=solo1
  ros2 launch mpc_control swarm_launch.py formation:=pair2
  ros2 launch mpc_control swarm_launch.py formation:=trio3
  ros2 launch mpc_control swarm_launch.py formation:=cross5
  ros2 launch mpc_control swarm_launch.py formation:=star5
  ros2 launch mpc_control swarm_launch.py formation:=grid9
"""
import math
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ── 出生位置（必须与各 start_N_px4.sh 的 POSES 完全一致）──────────────────
# 坐标系：PX4 NED（x=北/North, y=东/East, z=下/Down）
# Gazebo ENU (x=East,y=North) 换算：NED_x=ENU_y, NED_y=ENU_x
#
# ── solo1：单机诊断 ─────────────────────────────────────────────────────────
BIRTH_1 = [
    0.0,  0.0, 0.0,   # 0 中心
]

# ── pair2：双机前后纵列，间距 3 m ────────────────────────────────────────────
# 与 start_2_px4.sh POSES 对应:
#   drone 0: ENU(0, 0)   → NED( 0,  0) 中心
#   drone 1: ENU(0,-3)   → NED(-3,  0) 南 3 m
BIRTH_2 = [
     0.0,  0.0, 0.0,   # 0 中心
    -3.0,  0.0, 0.0,   # 1 南 (NED: x=North=-3)
]

# ── trio3：三机等边三角形，外接圆半径 3 m，边长 ≈ 5.196 m ──────────────────
# 与 start_3_px4.sh POSES 对应:
#   drone 0: ENU( 0,    3)    → NED(+3.000,  0.000) 北顶
#   drone 1: ENU( 2.598,-1.5) → NED(-1.500, +2.598) 东南
#   drone 2: ENU(-2.598,-1.5) → NED(-1.500, -2.598) 西南
BIRTH_3 = [
     3.0,    0.0,   0.0,   # 0 北顶
    -1.5,    2.598, 0.0,   # 1 东南
    -1.5,   -2.598, 0.0,   # 2 西南
]

# ── 5机 ────────────────────────────────────────────────────────────────────
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
# solo1 / pair2 / trio3：出生位置即编队偏移（领队在原点时各机的目标位置）
OFFSETS_SOLO1 = BIRTH_1[:]
OFFSETS_PAIR2 = BIRTH_2[:]
OFFSETS_TRIO3 = BIRTH_3[:]

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
# solo1: 传 [0]，mpc_node 会过滤掉 self.drone_id → 实际邻居列表为空
NBR_SOLO1 = [[0]]

# pair2: 互为唯一邻居
NBR_PAIR2 = [
    [1],   # 0 ↔ 1
    [0],   # 1 ↔ 0
]

# trio3: 全连接等边三角形（每机 2 个邻居）
NBR_TRIO3 = [
    [1, 2],   # 0 ↔ 东南、西南
    [0, 2],   # 1 ↔ 北顶、西南
    [0, 1],   # 2 ↔ 北顶、东南
]

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
    # ── 诊断队形（按 Phase 顺序验证）──────────────────────────────────────
    'solo1':  dict(num=1, birth=BIRTH_1, offsets=OFFSETS_SOLO1, nbr=NBR_SOLO1),
    'pair2':  dict(num=2, birth=BIRTH_2, offsets=OFFSETS_PAIR2, nbr=NBR_PAIR2),
    'trio3':  dict(num=3, birth=BIRTH_3, offsets=OFFSETS_TRIO3, nbr=NBR_TRIO3),
    # ── 正式编队 ───────────────────────────────────────────────────────────
    'cross5': dict(num=5, birth=BIRTH_5, offsets=OFFSETS_CROSS5, nbr=NBR_CROSS5),
    'star5':  dict(num=5, birth=BIRTH_5, offsets=OFFSETS_STAR5,  nbr=NBR_STAR5),
    'grid9':  dict(num=9, birth=BIRTH_9, offsets=OFFSETS_GRID9,  nbr=NBR_GRID9),
}

COMMON = {
    'target_alt':              -5.0,   # NED，负值=向上，-5.0 → 离地5米
    'max_speed':                3.0,   # 降低至3m/s：位置控制模式下更平滑，防止过冲
    'max_climb':                1.5,
    'max_accel':                4.0,
    'control_hz':              50.0,
    'neighbour_timeout':        2.0,   # 从0.5增至2.0：容忍通信延迟，减少错误降级
    'startup_zero_vel_frames':  100,   # 从50增至100：2s，给EKF更多收敛时间（多机尤其重要）
    'mpc_horizon':              20,
    'mpc_dt':                 0.05,
    'q_pos':                   4.0,
    'q_vel':                   2.0,
    'r_acc':                   0.1,
    'q_pos_terminal_scale':    2.0,
    'd_safe':                  1.5,
    'w_collision':           200.0,
    'w_formation':             0.5,
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
            'yaw_mode':    LaunchConfiguration('yaw_mode'),
            'start_x':      0.0,
            'start_y':      0.0,
            'altitude':     COMMON['target_alt'],
            'speed':        LaunchConfiguration('leader_speed'),
            'radius':       LaunchConfiguration('leader_radius'),
            'max_distance': LaunchConfiguration('max_distance'),
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
            description='solo1 | pair2 | trio3 | cross5 | star5 | grid9',
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
        DeclareLaunchArgument(
            'yaw_mode',
            default_value='fixed',
            description='偏航模式: fixed(固定朝向) | center(朝向圆心) | tangent(跟随方向)',
        ),
        DeclareLaunchArgument(
            'max_distance',
            default_value='20.0',
            description='直线模式最大飞行距离 (m)，到达后悬停',
        ),
        OpaqueFunction(function=_make_nodes),
    ])
