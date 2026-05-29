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
    def __init__(self, formation: str):
        super().__init__('diag_monitor')

        if formation not in FORMATION_CFG:
            self.get_logger().error(
                f'未知队形 "{formation}"，可选: {list(FORMATION_CFG.keys())}')
            raise ValueError(formation)

        self.cfg = FORMATION_CFG[formation]
        self.formation = formation
        self.num = self.cfg['num']
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

    def _make_pos_cb(self, idx):
        def cb(msg):
            d = self.drones[idx]
            d.pos = np.array([msg.x, msg.y, msg.z])
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


def main():
    parser = argparse.ArgumentParser(description='编队健康诊断监控')
    parser.add_argument('--formation', '-f', default='pair2',
                        choices=list(FORMATION_CFG.keys()),
                        help='队形名称')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    try:
        node = DiagMonitor(args.formation)
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
