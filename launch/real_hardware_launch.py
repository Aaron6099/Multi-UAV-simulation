#!/usr/bin/env python3
"""真机部署 launch — 每架机的 RPi 只启【本机】节点（与 SITL swarm_launch 一机全启不同）。

拓扑（pair2 真机）:
  drone0 RPi:  MicroXRCEAgent(串口连FC) + mpc_node(drone_id=0)
  drone1 RPi:  MicroXRCEAgent(串口连FC) + mpc_node(drone_id=1)
  地面站:      leader_node（编队参考 + 就绪门控）；diag_monitor.py --log 手动另开

用法:
  # drone0 的 RPi:
  ros2 launch mpc_control real_hardware_launch.py drone_id:=0 scenario:=S2_pair2_hover
  # drone1 的 RPi:
  ros2 launch mpc_control real_hardware_launch.py drone_id:=1 scenario:=S2_pair2_hover
  # 地面站（最后启动——leader 一发参考+就绪门控通过，编队即开动）:
  ros2 launch mpc_control real_hardware_launch.py role:=leader scenario:=S2_pair2_hover

一次性前置（每架）:
  FC(QGC):  PX4 v1.14；MAV_SYS_ID = drone_id+1；UXRCE_DDS_CFG=TEL1；
            SER_TEL1_BAUD=921600；COM_OBL_RC_ACT(offboard失联动作)按需；kill switch 必配。
            drone_id>=1 的 FC 还需 SD 卡 etc/extras.txt 设话题命名空间（对齐 SITL 约定，
            mpc_node 零改动）:
                uxrce_dds_client stop
                uxrce_dds_client start -t serial -d <TEL1设备> -b 921600 -n px4_<id>
            drone0 无命名空间（/fmu/...）。设备名用 `uxrce_dds_client status` 核对。
  RPi:      全员 ROS_DOMAIN_ID 一致 + CycloneDDS + chrony 时钟同步；
            CH340 udev 固定名建议 /dev/ttyFC。首次 launch 会现编 acados OCP
            （RPi4 数分钟），务必外场前在台架跑通一次。

与 SITL 的差异（本文件强制）:
  - scenario 带 faults（杀节点/通信注入）一律拒绝启动——真机不注入故障。
  - 默认 conservative:=true 保守限幅: max_speed≤1.5, max_climb≤1.0, max_accel≤2.0,
    d_safe≥2.5（与 scenario 取更保守一侧）。确有需要再显式 conservative:=false。
  - 默认 auto_arm:=false —— mpc_node 只发 setpoint 流并等待，由飞手 RC 解锁并切
    OFFBOARD（节点确认 nav_state/arming_state 后自动进入编队逻辑）。
  - alt_sync:=auto|true|false，默认 auto=沿用 yaml（SITL 关）。真机各机 home 海拔
    可能真不同，外场建议显式 alt_sync:=true。

起飞顺序（pair2）:
  1) 两架机摆到 scenarios.yaml births 标记点（NED，误差 < calib_max_origin_offset=2m）
  2) 各 RPi 启动本 launch → 等日志 "waiting RC ARM+OFFBOARD"
  3) 地面站启 role:=leader
  4) 飞手逐机 RC 解锁 → 切 OFFBOARD → 各机爬升至 target_alt 悬停成队
  5) leader 就绪门控（全员 pos_err<ready_pos_err 保持 ready_hold）通过后开动
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_LEADER_FALLBACK = dict(mode='hover', speed=0.5, radius=10.0,
                        max_distance=20.0, yaw_mode='fixed', start_delay=30.0)

# 真机保守限幅上限（conservative:=true 时与 scenario/defaults 取更保守一侧）
_CONSERVATIVE_CAPS = dict(max_speed=1.5, max_climb=1.0, max_accel=2.0)
_CONSERVATIVE_D_SAFE_FLOOR = 2.5


def _load_cfg():
    path = os.path.join(get_package_share_directory('mpc_control'),
                        'config', 'scenarios.yaml')
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def _flatten(nested):
    return [float(v) for pt in nested for v in pt]


def _make_nodes(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context).strip()

    cfg = _load_cfg()
    formations = cfg['formations']
    scenarios = cfg.get('scenarios', {})

    # ── 选 scenario / formation（与 swarm_launch 同逻辑）─────────────────────
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

    # ── 真机红线：故障注入场景一律拒绝 ───────────────────────────────────────
    if scen.get('faults'):
        raise ValueError(
            f'scenario "{scen_name}" 带 faults={scen["faults"]} —— '
            f'真机禁止故障注入，请改用无 faults 工况（如 S2_pair2_hover）')

    # ── 几何 ─────────────────────────────────────────────────────────────────
    births = scen.get('birth_override', fm['birth'])
    offsets = fm.get('offsets', fm['birth'])
    scale = scen.get('offsets_scale')
    if scale:
        offsets = [[v * float(scale) for v in pt] for pt in offsets]
    neighbours = fm['neighbours']
    num = len(births)
    birth_flat = _flatten(births)
    offsets_flat = _flatten(offsets)

    # ── 公共参数: defaults + scenario.limits + 真机保守限幅 ──────────────────
    common = dict(cfg.get('defaults', {}))
    for k, v in scen.get('limits', {}).items():
        common[k] = float(v)
    # 通信注入参数强制清零（双保险，faults 已拒绝）
    common['comms_delay_ms'] = 0.0
    common['comms_dropout'] = 0.0

    if arg('conservative').lower() != 'false':
        for k, cap in _CONSERVATIVE_CAPS.items():
            common[k] = min(float(common.get(k, cap)), cap)
        common['d_safe'] = max(float(common.get('d_safe', 1.5)),
                               _CONSERVATIVE_D_SAFE_FLOOR)

    alt_sync = arg('alt_sync').lower()
    if alt_sync in ('true', 'false'):
        v = (alt_sync == 'true')
        common['alt_sync_enable'] = v
        common['alt_resync_enable'] = v

    role = arg('role')
    nodes = []

    # ── role=leader: 地面站只跑 leader_node ──────────────────────────────────
    if role == 'leader':
        scen_leader = scen.get('leader', {})

        def resolve(arg_name, key, cast):
            a = arg(arg_name)
            if a != '':
                return cast(a)
            if key in scen_leader:
                return cast(scen_leader[key])
            return _LEADER_FALLBACK[key]

        nodes.append(Node(
            package='mpc_control', executable='leader_node', name='leader_node',
            output='screen', parameters=[{
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
                'ready_hold':    float(scen_leader.get('ready_hold', 5.0)),
                'ready_pos_err': float(scen_leader.get('ready_pos_err', 0.5)),
            }],
        ))
        return nodes

    # ── role=drone: 本机 XRCE agent + 本机 mpc_node ──────────────────────────
    drone_id = int(arg('drone_id'))
    if not 0 <= drone_id < num:
        raise ValueError(f'drone_id={drone_id} 越界（{formation} 共 {num} 机）')

    if arg('start_agent').lower() != 'false':
        nodes.append(ExecuteProcess(
            cmd=['MicroXRCEAgent', 'serial',
                 '--dev', arg('agent_dev'), '-b', arg('agent_baud')],
            output='screen', name='xrce_agent'))

    p = dict(common)
    p['drone_id'] = drone_id
    p['num_drones'] = num
    p['birth_positions_flat'] = birth_flat
    p['formation_offsets_flat'] = offsets_flat
    p['neighbours'] = [int(x) for x in neighbours[drone_id]]
    p['auto_arm_enable'] = (arg('auto_arm').lower() == 'true')
    nodes.append(Node(
        package='mpc_control', executable='mpc_node',
        name=f'mpc_node_{drone_id}', namespace=f'px4_{drone_id}',
        output='screen', parameters=[p],
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('role', default_value='drone',
                              description='drone=本机FC+mpc_node | leader=地面站参考节点'),
        DeclareLaunchArgument('drone_id', default_value='0',
                              description='本机编号（role:=drone 时必填，0..N-1）'),
        DeclareLaunchArgument('scenario', default_value='',
                              description='scenarios.yaml 工况名（faults 工况会被拒绝）'),
        DeclareLaunchArgument('formation', default_value='pair2',
                              description='scenario 未设时的队形'),
        DeclareLaunchArgument('auto_arm', default_value='false',
                              description='true=节点自动 ARM+OFFBOARD（仅台架）；'
                                          '默认 false=等飞手 RC 操作'),
        DeclareLaunchArgument('conservative', default_value='true',
                              description='真机保守限幅 v≤1.5 climb≤1.0 a≤2.0 d_safe≥2.5'),
        DeclareLaunchArgument('alt_sync', default_value='auto',
                              description='auto=沿用yaml | true | false（外场建议 true）'),
        DeclareLaunchArgument('start_agent', default_value='true',
                              description='随 launch 启动 MicroXRCEAgent'),
        DeclareLaunchArgument('agent_dev', default_value='/dev/ttyUSB0',
                              description='FC 串口设备（CH340=/dev/ttyUSB0，udev 固定名更稳）'),
        DeclareLaunchArgument('agent_baud', default_value='921600',
                              description='与 SER_TEL1_BAUD 一致'),
        # leader 覆盖项（role:=leader 时生效，同 swarm_launch）
        DeclareLaunchArgument('leader_mode', default_value=''),
        DeclareLaunchArgument('leader_speed', default_value=''),
        DeclareLaunchArgument('leader_radius', default_value=''),
        DeclareLaunchArgument('yaw_mode', default_value=''),
        DeclareLaunchArgument('max_distance', default_value=''),
        DeclareLaunchArgument('leader_start_delay', default_value=''),
        OpaqueFunction(function=_make_nodes),
    ])
