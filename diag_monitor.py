#!/usr/bin/env python3
"""
diag_monitor.py — 实时编队健康诊断监控

订阅所有无人机的位置、状态和 MPC 健康话题，每秒刷新显示：
  - 每机：高度误差、ARM/OFFBOARD 状态、速度
  - 编队：机间距（最小值高亮）、队形偏差
  - MPC：求解时间、降级次数、是否当前在 hover 降级
  - 安全违规计数（间距 < 阈值的事件数）

用法:
  ros2 run mpc_control diag_monitor --ros-args -p formation:=solo1
  ros2 run mpc_control diag_monitor --ros-args -p formation:=pair2
  ros2 run mpc_control diag_monitor --ros-args -p formation:=trio3
  ros2 run mpc_control diag_monitor --ros-args -p formation:=cross5
  ros2 run mpc_control diag_monitor --ros-args -p formation:=grid9

  # 也可直接运行（不在 ROS2 包里）：
  python3 diag_monitor.py --formation pair2
"""

import argparse
import math
import sys
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus

# ── 队形配置（与 swarm_launch.py 保持一致）──────────────────────────────────
FORMATION_CFG = {
    'solo1': {
        'num': 1,
        'offsets_ned': np.array([[0.0, 0.0, 0.0]]),
        'pairs': [],
        'target_alt': -5.0,
        'd_safe': 1.5,
    },
    'pair2': {
        'num': 2,
        'offsets_ned': np.array([[0.0, 0.0, 0.0], [-3.0, 0.0, 0.0]]),
        'pairs': [(0, 1)],
        'target_alt': -5.0,
        'd_safe': 1.5,
    },
    'trio3': {
        'num': 3,
        'offsets_ned': np.array([
            [ 3.0,    0.0,   0.0],
            [-1.5,    2.598, 0.0],
            [-1.5,   -2.598, 0.0],
        ]),
        'pairs': [(0, 1), (0, 2), (1, 2)],
        'target_alt': -5.0,
        'd_safe': 1.5,
    },
    'cross5': {
        'num': 5,
        'offsets_ned': np.array([
            [ 0.0,  0.0, 0.0],
            [ 0.0,  3.0, 0.0],
            [ 0.0, -3.0, 0.0],
            [ 3.0,  0.0, 0.0],
            [-3.0,  0.0, 0.0],
        ]),
        'pairs': [(0,1),(0,2),(0,3),(0,4)],
        'target_alt': -5.0,
        'd_safe': 1.5,
    },
    'star5': {
        'num': 5,
        'offsets_ned': np.array([
            [ 3.0,    0.0,    0.0],
            [ 0.927,  2.853,  0.0],
            [-2.427,  1.763,  0.0],
            [-2.427, -1.763,  0.0],
            [ 0.927, -2.853,  0.0],
        ]),
        'pairs': [(0,1),(1,2),(2,3),(3,4),(4,0)],
        'target_alt': -5.0,
        'd_safe': 1.5,
    },
    'grid9': {
        'num': 9,
        'offsets_ned': np.array([
            [ 0.0,  0.0, 0.0],
            [ 0.0,  3.0, 0.0],
            [ 0.0, -3.0, 0.0],
            [ 3.0,  0.0, 0.0],
            [-3.0,  0.0, 0.0],
            [ 3.0,  3.0, 0.0],
            [-3.0,  3.0, 0.0],
            [ 3.0, -3.0, 0.0],
            [-3.0, -3.0, 0.0],
        ]),
        'pairs': [(0,1),(0,2),(0,3),(0,4),(1,5),(1,6),(2,7),(2,8),(3,5),(3,7),(4,6),(4,8)],
        'target_alt': -5.0,
        'd_safe': 1.5,
    },
}

# ── 各机出生点（世界系 NED），与 swarm_launch.py 的 BIRTH_* / mpc_node 的 world_birth 一致 ──
# diag 订阅的 vehicle_local_position 是各机相对【自身出生点】的本地系；要在统一世界系里
# 比较机间距/队形偏差，必须加回出生点：world = local + birth。否则非原点出生的机（如 pair2
# 的 drone1 出生在 (-3,0)）本地读数≈0 会被误判成与中心机重叠 → 假 CRIT、队形误差虚高。
# ★ star5 出生在十字布局(BIRTH_5)、目标才是五边形(offsets)，故 birth ≠ offsets，不能用 offsets 顶替。
_BIRTH5 = np.array([
    [ 0.0,  0.0, 0.0],   # 0 中心
    [ 0.0,  3.0, 0.0],   # 1 东
    [ 0.0, -3.0, 0.0],   # 2 西
    [ 3.0,  0.0, 0.0],   # 3 北
    [-3.0,  0.0, 0.0],   # 4 南
])
BIRTH_NED = {
    'solo1':  np.array([[0.0, 0.0, 0.0]]),
    'pair2':  np.array([[0.0, 0.0, 0.0], [-3.0, 0.0, 0.0]]),
    'trio3':  np.array([[3.0, 0.0, 0.0], [-1.5, 2.598, 0.0], [-1.5, -2.598, 0.0]]),
    'cross5': _BIRTH5,
    'star5':  _BIRTH5,   # ★ 出生=十字，目标=五边形；birth ≠ offsets
    'grid9':  np.concatenate([_BIRTH5, np.array([
        [ 3.0,  3.0, 0.0],   # 5 东北
        [-3.0,  3.0, 0.0],   # 6 东南
        [ 3.0, -3.0, 0.0],   # 7 西北
        [-3.0, -3.0, 0.0],   # 8 西南
    ])]),
}

# 安全间距告警阈值（比 d_safe 多 0.3m 余量，给提前预警）
SPACING_WARN  = 1.8   # 黄色警告
SPACING_CRIT  = 1.5   # 红色危险（等于 d_safe）


def qos_sub():
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def topic_for(drone_id, suffix):
    if drone_id == 0:
        return f'/fmu/{suffix}'
    return f'/px4_{drone_id}/fmu/{suffix}'


def mpc_topic_for(drone_id, suffix):
    if drone_id == 0:
        return f'/mpc/{suffix}'
    return f'/px4_{drone_id}/mpc/{suffix}'


class DroneData:
    def __init__(self):
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.received = False
        self.last_stamp = 0.0
        self.arm_state  = 0   # 1=disarmed, 2=armed
        self.nav_state  = 0   # 14=offboard
        self.status_recv = False
        # MPC health
        self.mpc_status    = -1
        self.solve_ms      = 0.0
        self.fallback_count = 0
        self.hover_active  = False
        self.pos_err       = 0.0
        self.health_recv   = False


class DiagMonitor(Node):
    def __init__(self, formation: str, log_path: str = None):
        super().__init__('diag_monitor')

        if formation not in FORMATION_CFG:
            self.get_logger().error(
                f'未知队形 "{formation}"，可选: {list(FORMATION_CFG.keys())}')
            raise ValueError(formation)

        self.cfg = FORMATION_CFG[formation]
        self.formation = formation
        self.num = self.cfg['num']
        self.birth = BIRTH_NED[formation]   # (num,3) 世界系出生点；world = local + birth
        self.drones = [DroneData() for _ in range(self.num)]

        # 统计
        self._safety_violations = 0   # 间距 < SPACING_CRIT 的事件次数
        self._start_time = time.time()
        self._leader_pos = np.zeros(3)
        self._leader_vel = np.zeros(3)
        self._leader_recv = False

        q = qos_sub()
        for i in range(self.num):
            self.create_subscription(
                VehicleLocalPosition,
                topic_for(i, 'out/vehicle_local_position'),
                self._make_pos_cb(i), q,
            )
            self.create_subscription(
                VehicleStatus,
                topic_for(i, 'out/vehicle_status'),
                self._make_status_cb(i), q,
            )
            self.create_subscription(
                Float32MultiArray,
                mpc_topic_for(i, 'health'),
                self._make_health_cb(i), 10,
            )

        self.create_subscription(
            Float64MultiArray, '/leader/state', self._on_leader, 10)

        self.create_timer(1.0, self._print_status)
        self.get_logger().info(
            f'diag_monitor 已启动，监控队形={formation}，{self.num} 架无人机')

        # ── 可选：结构化飞行记录 (CSV)，每秒一行，供 analyze_flight.py 复盘 ──
        self._csv = None
        if log_path:
            import os
            d = os.path.dirname(os.path.abspath(log_path))
            if d:
                os.makedirs(d, exist_ok=True)
            self._csv = open(log_path, 'w', buffering=1)
            hdr = ['t']
            for i in range(self.num):
                # d{i}_x / d{i}_y 为世界系 NED 位置(北/东)，用于俯视轨迹图；
                # 追加在每机分组最前，纯增列、向后兼容 analyze_flight.py。
                hdr += [f'd{i}_x', f'd{i}_y', f'd{i}_z', f'd{i}_zerr',
                        f'd{i}_velxy', f'd{i}_arm',
                        f'd{i}_nav', f'd{i}_mpc', f'd{i}_solve_ms',
                        f'd{i}_fallback', f'd{i}_hover', f'd{i}_poserr']
            hdr += ['min_spacing', 'formation_max_err', 'safety_violations',
                    'total_fallbacks', 'max_solve_ms',
                    'leader_x', 'leader_y', 'leader_vx', 'leader_vy']
            self._csv.write(','.join(hdr) + '\n')
            self.get_logger().info(f'flight log → {log_path}')

    def _make_pos_cb(self, idx):
        def cb(msg):
            d = self.drones[idx]
            # local→world：加回出生点，与 mpc_node 的 ds.pos = local + world_birth 一致。
            # 用静态出生点：稳态正确；EKF 偶发 xy reset 后会有短暂偏差，不影响报告稳态指标。
            d.pos = np.array([msg.x, msg.y, msg.z]) + self.birth[idx]
            d.vel = np.array([msg.vx, msg.vy, msg.vz])
            d.received = True
            d.last_stamp = self.get_clock().now().nanoseconds * 1e-9
        return cb

    def _make_status_cb(self, idx):
        def cb(msg):
            d = self.drones[idx]
            d.arm_state   = msg.arming_state
            d.nav_state   = msg.nav_state
            d.status_recv = True
        return cb

    def _make_health_cb(self, idx):
        def cb(msg):
            if len(msg.data) < 6:
                return
            d = self.drones[idx]
            d.mpc_status     = int(msg.data[1])
            d.solve_ms       = float(msg.data[2])
            d.fallback_count = int(msg.data[3])
            d.hover_active   = bool(msg.data[4])
            d.pos_err        = float(msg.data[5])
            d.health_recv    = True
        return cb

    def _on_leader(self, msg):
        if len(msg.data) < 8:
            return
        self._leader_pos = np.array([msg.data[1], msg.data[2], msg.data[3]])
        self._leader_vel = np.array([msg.data[4], msg.data[5], msg.data[6]])
        self._leader_recv = True

    # ── 格式化辅助 ──────────────────────────────────────────────────────────
    @staticmethod
    def _arm_str(s):
        if s == 2: return '\033[32mARMED  \033[0m'
        if s == 1: return '\033[33mDISARMD\033[0m'
        return '\033[90mUNKNOWN\033[0m'

    @staticmethod
    def _nav_str(s):
        if s == 14: return '\033[32mOFFBOARD\033[0m'
        return f'\033[33mNAV={s:2d}  \033[0m'

    @staticmethod
    def _spacing_str(d):
        if d < SPACING_CRIT:
            return f'\033[31m{d:.2f}m<!CRIT\033[0m'
        if d < SPACING_WARN:
            return f'\033[33m{d:.2f}m<!WARN\033[0m'
        return f'\033[32m{d:.2f}m\033[0m'

    @staticmethod
    def _mpc_status_str(s):
        if s == 0: return '\033[32m OK \033[0m'
        if s == 2: return '\033[33mITER\033[0m'
        if s == -1: return '\033[90m--- \033[0m'
        return f'\033[31m ERR{s}\033[0m'

    def _print_status(self):
        now = time.time()
        elapsed = now - self._start_time
        h, rem = divmod(int(elapsed), 3600)
        m, s   = divmod(rem, 60)
        uptime = f'{h:02d}:{m:02d}:{s:02d}'
        ts = datetime.now().strftime('%H:%M:%S')

        lines = []
        lines.append(
            f'\033[2J\033[H'   # 清屏
            f'╔══ SWARM DIAG [{ts}] formation={self.formation} '
            f'uptime={uptime} ══╗'
        )

        # ── 领队 ──────────────────────────────────────────────────────────
        if self._leader_recv:
            lp = self._leader_pos
            lv = self._leader_vel
            lines.append(
                f'  Leader: pos=({lp[0]:+.2f}, {lp[1]:+.2f}, {lp[2]:+.2f})  '
                f'vel=({lv[0]:+.2f}, {lv[1]:+.2f})  \033[32m[RECV]\033[0m'
            )
        else:
            lines.append('  Leader: \033[31m[NO SIGNAL]\033[0m')

        lines.append('')

        # ── 每机状态 ──────────────────────────────────────────────────────
        target_alt = self.cfg['target_alt']
        for i, d in enumerate(self.drones):
            if not d.received:
                lines.append(f'  Drone {i}: \033[31m[NO POSITION]\033[0m')
                continue

            z_err = d.pos[2] - target_alt
            vel_xy = float(np.linalg.norm(d.vel[:2]))
            age = now - d.last_stamp

            # MPC
            if d.health_recv:
                mpc_info = (
                    f'MPC={self._mpc_status_str(d.mpc_status)} '
                    f'solve={d.solve_ms:.1f}ms '
                    f'fallback={d.fallback_count}'
                    + ('\033[33m[HOVER]\033[0m' if d.hover_active else '')
                )
            else:
                mpc_info = '\033[90mMPC=[no diag topic]\033[0m'

            lines.append(
                f'  Drone {i}: '
                f'z={d.pos[2]:+.2f}m(err={z_err:+.2f}m)  '
                f'velXY={vel_xy:.2f}m/s  '
                f'{self._arm_str(d.arm_state)}  '
                f'{self._nav_str(d.nav_state)}  '
                f'age={age:.1f}s  '
                f'{mpc_info}'
            )

        lines.append('')

        # ── 机间距 ────────────────────────────────────────────────────────
        pairs = self.cfg['pairs']
        if pairs:
            min_spacing = float('inf')
            spacing_strs = []
            for (i, j) in pairs:
                di, dj = self.drones[i], self.drones[j]
                if di.received and dj.received:
                    dist = float(np.linalg.norm(di.pos - dj.pos))
                    if dist < SPACING_CRIT:
                        self._safety_violations += 1
                    min_spacing = min(min_spacing, dist)
                    spacing_strs.append(
                        f'{i}↔{j}: {self._spacing_str(dist)}')
                else:
                    spacing_strs.append(f'{i}↔{j}: \033[90m---\033[0m')

            lines.append('  Spacing: ' + '  '.join(spacing_strs))
            if math.isfinite(min_spacing):
                lines.append(
                    f'  Min spacing: {self._spacing_str(min_spacing)}'
                    f'  (warn<{SPACING_WARN}m, crit<{SPACING_CRIT}m)'
                    f'  safety_violations={self._safety_violations}'
                )
        else:
            lines.append('  Spacing: N/A (solo1)')

        # ── 队形偏差 ──────────────────────────────────────────────────────
        offsets = self.cfg['offsets_ned']
        leader_xy = self._leader_pos[:2] if self._leader_recv else np.zeros(2)
        all_received = all(d.received for d in self.drones)
        if all_received and self._leader_recv:
            form_errs = []
            for i, d in enumerate(self.drones):
                expected_xy = leader_xy + offsets[i, :2]
                err = float(np.linalg.norm(d.pos[:2] - expected_xy))
                form_errs.append(err)
            max_err = max(form_errs)
            err_str = '  '.join(f'd{i}:{e:.2f}m' for i, e in enumerate(form_errs))
            color = '\033[32m' if max_err < 0.5 else ('\033[33m' if max_err < 1.0 else '\033[31m')
            lines.append(f'  Formation err: {err_str}  {color}max={max_err:.2f}m\033[0m')

        lines.append('')

        # ── Gate 状态摘要 ─────────────────────────────────────────────────
        all_armed    = all(d.arm_state == 2 for d in self.drones if d.status_recv)
        all_offboard = all(d.nav_state == 14 for d in self.drones if d.status_recv)
        any_hover    = any(d.hover_active for d in self.drones if d.health_recv)
        total_fb     = sum(d.fallback_count for d in self.drones)
        max_solve_ms = max((d.solve_ms for d in self.drones if d.health_recv), default=0.0)

        g_arm  = '\033[32m✓\033[0m' if all_armed    else '\033[31m✗\033[0m'
        g_ofb  = '\033[32m✓\033[0m' if all_offboard else '\033[31m✗\033[0m'
        g_mpc  = '\033[32m✓\033[0m' if not any_hover else '\033[33m!\033[0m'
        g_safe = '\033[32m✓\033[0m' if self._safety_violations == 0 else '\033[31m✗\033[0m'

        lines.append(
            f'  Gate checks: ARM={g_arm}  OFFBOARD={g_ofb}  '
            f'MPC_ok={g_mpc}(fallbacks={total_fb})  '
            f'SAFE={g_safe}(violations={self._safety_violations})  '
            f'max_solve={max_solve_ms:.1f}ms'
        )
        lines.append('╚' + '═' * 70 + '╝')

        print('\n'.join(lines), flush=True)
        self._log_row(elapsed)


    def _log_row(self, elapsed):
        if self._csv is None:
            return
        nan = float('nan')
        row = [f'{elapsed:.1f}']
        for d in self.drones:
            if d.received:
                x = float(d.pos[0])
                y = float(d.pos[1])
                z = float(d.pos[2])
                zerr = z - self.cfg['target_alt']
                velxy = float(np.linalg.norm(d.vel[:2]))
            else:
                x = y = z = zerr = velxy = nan
            row += [f'{x:.3f}', f'{y:.3f}', f'{z:.3f}', f'{zerr:.3f}',
                    f'{velxy:.3f}',
                    str(d.arm_state), str(d.nav_state), str(d.mpc_status),
                    f'{d.solve_ms:.3f}', str(d.fallback_count),
                    str(int(d.hover_active)), f'{d.pos_err:.3f}']
        min_sp = nan
        if self.cfg['pairs']:
            dmin = float('inf')
            for (i, j) in self.cfg['pairs']:
                di, dj = self.drones[i], self.drones[j]
                if di.received and dj.received:
                    dmin = min(dmin, float(np.linalg.norm(di.pos - dj.pos)))
            if math.isfinite(dmin):
                min_sp = dmin
        max_ferr = nan
        if self._leader_recv and all(d.received for d in self.drones):
            offs = self.cfg['offsets_ned']
            lxy = self._leader_pos[:2]
            max_ferr = max(
                float(np.linalg.norm(self.drones[i].pos[:2] - (lxy + offs[i, :2])))
                for i in range(self.num))
        total_fb  = sum(d.fallback_count for d in self.drones)
        max_solve = max((d.solve_ms for d in self.drones if d.health_recv), default=0.0)
        if self._leader_recv:
            lx, ly   = float(self._leader_pos[0]), float(self._leader_pos[1])
            lvx, lvy = float(self._leader_vel[0]), float(self._leader_vel[1])
        else:
            lx = ly = lvx = lvy = nan
        row += [f'{min_sp:.3f}', f'{max_ferr:.3f}', str(self._safety_violations),
                str(total_fb), f'{max_solve:.3f}',
                f'{lx:.3f}', f'{ly:.3f}', f'{lvx:.3f}', f'{lvy:.3f}']
        self._csv.write(','.join(row) + '\n')


def main():
    parser = argparse.ArgumentParser(description='编队健康诊断监控')
    parser.add_argument('--formation', '-f', default='pair2',
                        choices=list(FORMATION_CFG.keys()),
                        help='队形名称')
    parser.add_argument('--log', nargs='?', const='', default=None,
                        help='把每秒指标写入 CSV：给路径用之，省略路径则自动命名 '
                             'flight_<formation>_<时间戳>.csv')
    args, ros_args = parser.parse_known_args()

    log_path = None
    if args.log is not None:
        log_path = args.log or \
            f'flight_{args.formation}_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'

    rclpy.init(args=ros_args)
    try:
        node = DiagMonitor(args.formation, log_path)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError:
        sys.exit(1)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
