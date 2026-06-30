#!/usr/bin/env python3
"""启动 MPC 编队。队形几何 / 参数 / 测试工况统一来自 config/scenarios.yaml
（单一真值源；与 start_N 经 tools/gen_spawn.py 读同一份 birth，杜绝漂移）。

两种用法:
  # 1) 按队形（向后兼容旧命令）——几何取 scenarios.yaml 的 formations.<name>
  ros2 launch mpc_control swarm_launch.py formation:=cross5 \
       leader_mode:=line leader_speed:=1.0 max_distance:=20.0

  # 2) 按命名工况——formation/leader 默认取该工况，仍可用 key:=value 覆盖
  ros2 launch mpc_control swarm_launch.py scenario:=S4_cross5_line
  ros2 launch mpc_control swarm_launch.py scenario:=S11_cross5_perturbed   # 扰动出生

参数优先级（高→低）: 显式 key:=value 启动参数  >  scenario 设定  >  内置默认。

⚠️ 改了 config/scenarios.yaml 后需 `colcon build`（launch 读的是 install 下的安装副本；
   start_N 走 gen_spawn 读源码副本，build 后两者一致）。
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction, TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# leader 在既无启动参数、又无 scenario 设定时的内置默认（与旧 DeclareLaunchArgument 一致）
_LEADER_FALLBACK = dict(mode='hover', speed=0.5, radius=10.0,
                        max_distance=20.0, yaw_mode='fixed', start_delay=30.0)


def _load_cfg():
    path = os.path.join(get_package_share_directory('mpc_control'),
                        'config', 'scenarios.yaml')
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def _flatten(nested):
    """[[N,E,D],...] → 扁平 float 列表（mpc_node 的 *_flat 是 DOUBLE_ARRAY，必须 float）。"""
    return [float(v) for pt in nested for v in pt]


def _make_nodes(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context).strip()

    cfg = _load_cfg()
    formations = cfg['formations']
    scenarios = cfg.get('scenarios', {})

    # ── 选 scenario / formation ──────────────────────────────────────────────
    scen_name = arg('scenario')
    if scen_name:
        if scen_name not in scenarios:
            raise ValueError(f'未知 scenario "{scen_name}"，可选: {list(scenarios)}')
        scen = scenarios[scen_name]
        formation = scen['formation']
    else:
        scen = {}
        formation = arg('formation')
    if formation not in formations:
        raise ValueError(f'未知 formation "{formation}"，可选: {list(formations)}')
    fm = formations[formation]

    # ── 几何：birth / offsets / neighbours（scenario 可覆盖 birth/scale）──────
    births = scen.get('birth_override', fm['birth'])
    offsets = fm.get('offsets', fm['birth'])
    scale = scen.get('offsets_scale')
    if scale:
        offsets = [[v * float(scale) for v in pt] for pt in offsets]
    neighbours = fm['neighbours']
    num = len(births)
    birth_flat = _flatten(births)
    offsets_flat = _flatten(offsets)

    # ── MPC 公共参数：defaults + formation 默认 + scenario.limits 覆盖 ─────────
    common = dict(cfg.get('defaults', {}))

    # ── 5/9 机高度拉齐：开 Tier2 alt re-sync 并加快收敛（先设，limits 可覆盖）──
    # SITL 多 PX4 实例 baro/EKF 的 ref_alt 各自温漂，不拉齐则各机控到 local z=-5
    # 时真实高度散 ~1.5m。drone0 广播 ref_alt 基准、各机限速纠 world_birth_z。
    # rate 0.05→0.2(补 0.5m 偏差 10s→2.5s)、EMA alpha 0.05→0.1(滤波 0.4s→0.2s)。
    # 实测 5 机悬停真高散 1.5m→0.18m。2/3 机漂移小、不开。
    if formation in ('cross5', 'star5', 'grid9'):
        common['alt_resync_enable'] = True
        common['alt_resync_rate'] = 0.2
        common['alt_ref_filter_alpha'] = 0.1

    # scenario.limits 最高优先级（后设，可覆盖 formation 默认值）
    # bool 项（如 alt_resync_enable）保留 bool，否则 ROS2 参数类型不匹配；其余转 float
    for k, v in scen.get('limits', {}).items():
        common[k] = v if isinstance(v, bool) else float(v)

    # ── P2 故障注入：scenario.faults（S14 杀节点 / S16 通信劣化）──────────────
    faults = scen.get('faults', {})
    if 'comms_delay_ms' in faults:
        common['comms_delay_ms'] = float(faults['comms_delay_ms'])
    if 'comms_dropout' in faults:
        common['comms_dropout'] = float(faults['comms_dropout'])

    # ── leader 参数：显式启动参数(非空) > scenario.leader > 内置默认 ──────────
    scen_leader = scen.get('leader', {})

    def resolve(arg_name, key, cast):
        a = arg(arg_name)
        if a != '':
            return cast(a)
        if key in scen_leader:
            return cast(scen_leader[key])
        return _LEADER_FALLBACK[key]

    leader_params = {
        'mode':         resolve('leader_mode', 'mode', str),
        'yaw_mode':     resolve('yaw_mode', 'yaw_mode', str),
        'start_x':      0.0,
        'start_y':      0.0,
        'altitude':     float(common['target_alt']),
        'speed':        resolve('leader_speed', 'speed', float),
        'radius':       resolve('leader_radius', 'radius', float),
        'max_distance': resolve('max_distance', 'max_distance', float),
        'start_delay':  resolve('leader_start_delay', 'start_delay', float),
        'publish_hz':   50.0,
        'num_drones':   num,
        # 就绪门控阈值：scenario.leader 可覆盖（默认同 leader_node）。
        # 加大 ready_hold 可让僚机在静止悬停下把队形畸变收敛到位再开动（先成型再走）。
        'ready_hold':    float(scen_leader.get('ready_hold', 2.0)),
        'ready_pos_err': float(scen_leader.get('ready_pos_err', 0.5)),
    }

    # 若 scenario/limits 未显式设置安全飞散阈值，从 leader max_distance 自动推导
    # (默认 5m 对追线场景太紧；leader 最远 30m 时阈值自动升到 36m)
    if 'safety_max_track_dist' not in scen.get('limits', {}):
        md = leader_params['max_distance']
        if md > 5.0:
            common.setdefault('safety_max_track_dist', round(md * 1.2, 1))

    nodes = [Node(
        package='mpc_control', executable='leader_node', name='leader_node',
        output='screen', parameters=[leader_params],
    )]

    for drone_id in range(num):
        p = dict(common)
        p['drone_id'] = drone_id
        p['num_drones'] = num
        p['birth_positions_flat'] = birth_flat
        p['formation_offsets_flat'] = offsets_flat
        p['neighbours'] = [int(x) for x in neighbours[drone_id]]
        nodes.append(Node(
            package='mpc_control', executable='mpc_node',
            name=f'mpc_node_{drone_id}', namespace=f'px4_{drone_id}',
            output='screen', parameters=[p],
        ))

    # ── S14 杀节点：kill_at_s 秒后 pkill 目标 mpc_node 进程（模拟整机失联）────
    if 'kill_drone' in faults:
        kill_id = int(faults['kill_drone'])
        kill_at = float(faults.get('kill_at_s', 30.0))
        if not 0 <= kill_id < num:
            raise ValueError(f'faults.kill_drone={kill_id} 越界 (num_drones={num})')
        nodes.append(TimerAction(period=kill_at, actions=[
            LogInfo(msg=f'[P2 FAULT INJECTION] t={kill_at:.0f}s — '
                        f'killing mpc_node_{kill_id} (drone {kill_id} 整机失联模拟)'),
            ExecuteProcess(
                cmd=['pkill', '-f', f'__node:=mpc_node_{kill_id}'],
                output='screen'),
        ]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario', default_value='',
            description='scenarios.yaml 里的工况名(如 S4_cross5_line)；'
                        '设了它则 formation/leader 默认从该工况取'),
        DeclareLaunchArgument(
            'formation', default_value='cross5',
            description='solo1|pair2|trio3|cross5|star5|grid9（scenario 未设时生效）'),
        DeclareLaunchArgument(
            'leader_mode', default_value='',
            description='hover|circle|line（留空=用 scenario/默认 hover）'),
        DeclareLaunchArgument(
            'leader_speed', default_value='',
            description='circle/line 速度 m/s（留空=用 scenario/默认 0.5）'),
        DeclareLaunchArgument(
            'leader_radius', default_value='',
            description='circle 半径 m（留空=用 scenario/默认 10）'),
        DeclareLaunchArgument(
            'yaw_mode', default_value='',
            description='fixed|center|tangent（留空=用 scenario/默认 fixed）'),
        DeclareLaunchArgument(
            'max_distance', default_value='',
            description='line 最大距离 m（留空=用 scenario/默认 20）'),
        DeclareLaunchArgument(
            'leader_start_delay', default_value='',
            description='起飞等待 s（留空=默认 30；就绪门控开启时此为兜底）'),
        OpaqueFunction(function=_make_nodes),
    ])
