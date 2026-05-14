#!/usr/bin/env python3
"""
Flocking experiment plotter.

Reads a ROS2 bag (recorded with `ros2 bag record`) containing PX4 telemetry
from N drones, plus the virtual leader state, and produces:

  1. Overlay plots (all drones on one figure):
       - position_xyz_overlay.png    (x, y, z vs time, 3 subplots)
       - velocity_xyz_overlay.png    (vx, vy, vz vs time, 3 subplots)
       - attitude_rpy_overlay.png    (roll, pitch, yaw vs time, 3 subplots)
       - trajectory_xy_overlay.png   (x-y top-down view, all drones + leader)

  2. Per-drone plots (one figure per drone, 4 subplots inside):
       - drone_<i>_summary.png       (pos, vel, att, traj in 2x2 grid)

  3. CSV exports for re-plotting in MATLAB if desired:
       - drone_<i>.csv               (time, x, y, z, vx, vy, vz, roll, pitch, yaw)
       - leader.csv                  (time, x, y, z, vx, vy, vz, yaw)

Usage:
    python3 plot_flocking.py <bag_dir>           # auto-detect num_drones
    python3 plot_flocking.py <bag_dir> -n 5      # explicit num_drones

Output goes to <bag_dir>/plots/ by default.
"""

import argparse
import math
import os
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')  # headless safe
import matplotlib.pyplot as plt

# ----- ROS2 bag reading -----
try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError as e:
    print('[ERROR] rosbag2_py / rclpy not available. Source ROS2 first:')
    print('   source /opt/ros/humble/setup.bash')
    print(f'   ({e})')
    sys.exit(1)


# ----------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------

def quat_to_rpy(w, x, y, z):
    """Quaternion (w, x, y, z) -> (roll, pitch, yaw) in radians (ZYX convention)."""
    # Roll
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # Pitch
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    # Yaw
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def topic_for_drone(prefix_func, drone_id, suffix):
    """Build topic name. drone 0 -> /fmu/...; drone N -> /px4_N/fmu/..."""
    if drone_id == 0:
        return f'/fmu/{suffix}'
    return f'/px4_{drone_id}/fmu/{suffix}'


# ----------------------------------------------------------------------
# Bag reader
# ----------------------------------------------------------------------

def read_bag(bag_dir, num_drones):
    """Reads the bag and returns:
        drones[i] = {'pos': DataFrame, 'att': DataFrame}
        leader    = DataFrame  (or None)
    """
    storage = StorageOptions(uri=str(bag_dir), storage_id='sqlite3')
    converter = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader = SequentialReader()
    reader.open(storage, converter)

    # Build topic -> type map
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    print(f'[info] bag has {len(topic_types)} topics:')
    for t, ty in sorted(topic_types.items()):
        print(f'        {t}  ({ty})')

    # Map: topic -> (drone_id or "leader", what)
    topic_map = {}
    for i in range(num_drones):
        pos_topic = topic_for_drone(None, i, 'out/vehicle_local_position')
        att_topic = topic_for_drone(None, i, 'out/vehicle_attitude')
        if pos_topic in topic_types:
            topic_map[pos_topic] = (i, 'pos')
        if att_topic in topic_types:
            topic_map[att_topic] = (i, 'att')

    if '/leader/state' in topic_types:
        topic_map['/leader/state'] = ('leader', 'leader')

    # Storage
    drone_pos = {i: [] for i in range(num_drones)}
    drone_att = {i: [] for i in range(num_drones)}
    leader_rows = []

    # Lazy message class cache
    msg_class_cache = {}

    def msg_class(type_str):
        if type_str not in msg_class_cache:
            msg_class_cache[type_str] = get_message(type_str)
        return msg_class_cache[type_str]

    # Iterate
    n_msgs = 0
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        if topic not in topic_map:
            continue
        n_msgs += 1
        type_str = topic_types[topic]
        cls = msg_class(type_str)
        msg = deserialize_message(raw, cls)
        who, what = topic_map[topic]

        if what == 'pos':
            # PX4 timestamp is in microseconds; we use it as the canonical time.
            drone_pos[who].append({
                't_us': msg.timestamp,
                'x': msg.x,
                'y': msg.y,
                'z': msg.z,
                'vx': msg.vx,
                'vy': msg.vy,
                'vz': msg.vz,
                'heading': msg.heading,
            })
        elif what == 'att':
            roll, pitch, yaw = quat_to_rpy(
                msg.q[0], msg.q[1], msg.q[2], msg.q[3]
            )
            drone_att[who].append({
                't_us': msg.timestamp,
                'roll': roll,
                'pitch': pitch,
                'yaw': yaw,
                'qw': msg.q[0], 'qx': msg.q[1], 'qy': msg.q[2], 'qz': msg.q[3],
            })
        elif what == 'leader':
            # Float32MultiArray layout written by virtual_leader_node:
            # [t, x, y, z, vx, vy, vz, yaw]
            d = list(msg.data)
            if len(d) >= 8:
                leader_rows.append({
                    't_s': d[0],
                    'x': d[1], 'y': d[2], 'z': d[3],
                    'vx': d[4], 'vy': d[5], 'vz': d[6],
                    'yaw': d[7],
                    't_bag_ns': t_ns,
                })

    print(f'[info] consumed {n_msgs} relevant messages')

    # ---- Convert to DataFrames and normalize time ----
    drones = {}
    # IMPORTANT: PX4 v1.14 instances may use DIFFERENT time bases:
    #   - some instances publish timestamps as UTC microseconds (huge number ~1.7e18)
    #   - others publish as boot-relative microseconds (small number ~ seconds since launch)
    # We can't safely use a single global t0 — drone 2 with boot-relative time
    # would land at t=0 while others sit at t=very_large_number.
    # Solution: normalize each drone independently to its own min(pos, att) timestamp.
    # All drones start within seconds of each other, so per-drone t=0 produces
    # visually-aligned overlay plots.

    for i in range(num_drones):
        # Find this drone's t0 from its own data (pos + att, whichever is earliest)
        own_origins = []
        if drone_pos[i]:
            own_origins.append(min(r['t_us'] for r in drone_pos[i]))
        if drone_att[i]:
            own_origins.append(min(r['t_us'] for r in drone_att[i]))
        if own_origins:
            my_t0_us = min(own_origins)
        else:
            my_t0_us = 0

        pos_df = pd.DataFrame(drone_pos[i])
        att_df = pd.DataFrame(drone_att[i])
        if not pos_df.empty:
            pos_df['t'] = (pos_df['t_us'] - my_t0_us) * 1e-6
            pos_df = pos_df[pos_df['t'] >= 0]
        if not att_df.empty:
            att_df['t'] = (att_df['t_us'] - my_t0_us) * 1e-6
            att_df = att_df[att_df['t'] >= 0]
        drones[i] = {'pos': pos_df, 'att': att_df, 't0_us': my_t0_us}
        print(f'[info] drone {i}: {len(pos_df)} pos rows, {len(att_df)} att rows, '
              f't0={my_t0_us}')

    # Pick a global t0 for the leader (use min over all drones, ignoring outliers).
    # Median is more robust than min in case one drone has an exotic boot-time.
    drone_t0s = [drones[i]['t0_us'] for i in range(num_drones) if drones[i]['t0_us']]
    if drone_t0s:
        # Use the largest cluster: PX4 instances synced to UTC will share huge values,
        # boot-time outliers are smaller. Median picks the cluster.
        global_t0_us = int(np.median(drone_t0s))
    else:
        global_t0_us = 0

    leader_df = pd.DataFrame(leader_rows)
    if not leader_df.empty:
        # Leader publishes its own t in seconds; align to same t0 if possible.
        # If leader's t is not aligned (it's wall-clock seconds maybe), prefer bag t.
        leader_df['t'] = (leader_df['t_bag_ns'] - global_t0_us * 1000) * 1e-9
        leader_df = leader_df[leader_df['t'] >= 0]
        print(f'[info] leader: {len(leader_df)} rows')
    else:
        print('[info] leader: no data')

    return drones, leader_df, num_drones


# ----------------------------------------------------------------------
# Plotting
# ----------------------------------------------------------------------

# Distinct colors for up to 9 drones
DRONE_COLORS = [
    '#1f77b4',  # 0 blue
    '#ff7f0e',  # 1 orange
    '#2ca02c',  # 2 green
    '#d62728',  # 3 red
    '#9467bd',  # 4 purple
    '#8c564b',  # 5 brown
    '#e377c2',  # 6 pink
    '#7f7f7f',  # 7 gray
    '#bcbd22',  # 8 olive
]

LEADER_COLOR = 'black'


def plot_position_overlay(drones, num_drones, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    labels = ['x (north) [m]', 'y (east) [m]', 'z (down) [m]']
    keys = ['x', 'y', 'z']
    for ax, key, lbl in zip(axes, keys, labels):
        for i in range(num_drones):
            df = drones[i]['pos']
            if df.empty:
                continue
            ax.plot(df['t'].to_numpy(), df[key].to_numpy(),
                    color=DRONE_COLORS[i % 9],
                    label=f'drone {i}', linewidth=1.2)
        ax.set_ylabel(lbl)
        ax.grid(True, alpha=0.3)
    axes[0].legend(loc='upper right', ncol=min(num_drones, 5), fontsize=9)
    axes[-1].set_xlabel('time [s]')
    fig.suptitle('Position (NED, world frame after birth offset)')
    fig.tight_layout()
    out = out_dir / 'position_xyz_overlay.png'
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f'[saved] {out}')


def plot_velocity_overlay(drones, num_drones, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    keys = ['vx', 'vy', 'vz']
    labels = ['vx [m/s]', 'vy [m/s]', 'vz [m/s]']
    for ax, key, lbl in zip(axes, keys, labels):
        for i in range(num_drones):
            df = drones[i]['pos']
            if df.empty:
                continue
            ax.plot(df['t'].to_numpy(), df[key].to_numpy(),
                    color=DRONE_COLORS[i % 9],
                    label=f'drone {i}', linewidth=1.2)
        ax.set_ylabel(lbl)
        ax.grid(True, alpha=0.3)
        ax.axhline(0.0, color='gray', linewidth=0.5)
    axes[0].legend(loc='upper right', ncol=min(num_drones, 5), fontsize=9)
    axes[-1].set_xlabel('time [s]')
    fig.suptitle('Velocity (NED)')
    fig.tight_layout()
    out = out_dir / 'velocity_xyz_overlay.png'
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f'[saved] {out}')


def plot_attitude_overlay(drones, num_drones, out_dir):
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    keys = ['roll', 'pitch', 'yaw']
    labels = ['roll [deg]', 'pitch [deg]', 'yaw [deg]']
    for ax, key, lbl in zip(axes, keys, labels):
        for i in range(num_drones):
            df = drones[i]['att']
            if df.empty:
                continue
            ax.plot(df['t'].to_numpy(), np.degrees(df[key].to_numpy()),
                    color=DRONE_COLORS[i % 9],
                    label=f'drone {i}', linewidth=1.2)
        ax.set_ylabel(lbl)
        ax.grid(True, alpha=0.3)
        ax.axhline(0.0, color='gray', linewidth=0.5)
    axes[0].legend(loc='upper right', ncol=min(num_drones, 5), fontsize=9)
    axes[-1].set_xlabel('time [s]')
    fig.suptitle('Attitude (Euler angles, ZYX)')
    fig.tight_layout()
    out = out_dir / 'attitude_rpy_overlay.png'
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f'[saved] {out}')


def plot_trajectory_overlay(drones, num_drones, leader_df, out_dir):
    fig, ax = plt.subplots(figsize=(10, 10))
    # NED: x=north, y=east. Plot with x as horizontal axis but actually
    # convention for "top-down view": east on horizontal, north on vertical.
    # We'll do (y_east) on x-axis and (x_north) on y-axis -> standard map view.
    for i in range(num_drones):
        df = drones[i]['pos']
        if df.empty:
            continue
        y_arr = df['y'].to_numpy()
        x_arr = df['x'].to_numpy()
        ax.plot(y_arr, x_arr, color=DRONE_COLORS[i % 9],
                label=f'drone {i}', linewidth=1.3)
        # Mark start (circle) and end (square)
        ax.plot(y_arr[0], x_arr[0], marker='o',
                markersize=10, color=DRONE_COLORS[i % 9],
                markerfacecolor='white', markeredgewidth=1.8, zorder=5)
        ax.plot(y_arr[-1], x_arr[-1], marker='s',
                markersize=10, color=DRONE_COLORS[i % 9], zorder=5)

    # Leader
    if leader_df is not None and not leader_df.empty:
        ly = leader_df['y'].to_numpy()
        lx = leader_df['x'].to_numpy()
        ax.plot(ly, lx, color=LEADER_COLOR,
                linestyle='--', linewidth=1.6, label='leader', alpha=0.8)
        ax.plot(ly[0], lx[0], marker='o',
                markersize=11, color=LEADER_COLOR,
                markerfacecolor='white', markeredgewidth=1.8, zorder=5)
        ax.plot(ly[-1], lx[-1], marker='s',
                markersize=11, color=LEADER_COLOR, zorder=5)

    ax.set_xlabel('y east [m]')
    ax.set_ylabel('x north [m]')
    ax.set_title('Horizontal trajectory  (○ = start, □ = end)')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=9, ncol=2)
    fig.tight_layout()
    out = out_dir / 'trajectory_xy_overlay.png'
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f'[saved] {out}')


def plot_per_drone(drones, num_drones, leader_df, out_dir):
    """One PNG per drone: 2x2 grid (pos, vel, attitude, trajectory)."""
    for i in range(num_drones):
        pos = drones[i]['pos']
        att = drones[i]['att']
        if pos.empty and att.empty:
            print(f'[skip] drone {i} has no data')
            continue

        fig, axes = plt.subplots(2, 2, figsize=(13, 9))
        c = DRONE_COLORS[i % 9]

        # (0,0) Position
        ax = axes[0][0]
        if not pos.empty:
            t_arr = pos['t'].to_numpy()
            ax.plot(t_arr, pos['x'].to_numpy(), label='x (north)', color='C0')
            ax.plot(t_arr, pos['y'].to_numpy(), label='y (east)', color='C1')
            ax.plot(t_arr, pos['z'].to_numpy(), label='z (down)', color='C2')
        ax.set_xlabel('time [s]')
        ax.set_ylabel('position [m]')
        ax.set_title('Position (NED)')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=9)

        # (0,1) Velocity
        ax = axes[0][1]
        if not pos.empty:
            t_arr = pos['t'].to_numpy()
            vx = pos['vx'].to_numpy()
            vy = pos['vy'].to_numpy()
            vz = pos['vz'].to_numpy()
            ax.plot(t_arr, vx, label='vx', color='C0')
            ax.plot(t_arr, vy, label='vy', color='C1')
            ax.plot(t_arr, vz, label='vz', color='C2')
            # Speed magnitude
            speed = np.sqrt(vx**2 + vy**2 + vz**2)
            ax.plot(t_arr, speed, label='|v|', color='black',
                    linestyle=':', linewidth=1.5)
        ax.set_xlabel('time [s]')
        ax.set_ylabel('velocity [m/s]')
        ax.set_title('Velocity (NED)')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=9)
        ax.axhline(0.0, color='gray', linewidth=0.5)

        # (1,0) Attitude
        ax = axes[1][0]
        if not att.empty:
            t_arr = att['t'].to_numpy()
            ax.plot(t_arr, np.degrees(att['roll'].to_numpy()),
                    label='roll', color='C0')
            ax.plot(t_arr, np.degrees(att['pitch'].to_numpy()),
                    label='pitch', color='C1')
            ax.plot(t_arr, np.degrees(att['yaw'].to_numpy()),
                    label='yaw', color='C2')
        ax.set_xlabel('time [s]')
        ax.set_ylabel('angle [deg]')
        ax.set_title('Attitude (Euler ZYX)')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=9)
        ax.axhline(0.0, color='gray', linewidth=0.5)

        # (1,1) Trajectory (this drone + leader for context)
        ax = axes[1][1]
        if not pos.empty:
            y_arr = pos['y'].to_numpy()
            x_arr = pos['x'].to_numpy()
            ax.plot(y_arr, x_arr, color=c,
                    label=f'drone {i}', linewidth=1.5)
            ax.plot(y_arr[0], x_arr[0], marker='o',
                    markersize=10, color=c,
                    markerfacecolor='white', markeredgewidth=1.6, zorder=5)
            ax.plot(y_arr[-1], x_arr[-1], marker='s',
                    markersize=10, color=c, zorder=5)
        if leader_df is not None and not leader_df.empty:
            ax.plot(leader_df['y'].to_numpy(), leader_df['x'].to_numpy(),
                    color=LEADER_COLOR,
                    linestyle='--', linewidth=1.3, label='leader', alpha=0.7)
        ax.set_xlabel('y east [m]')
        ax.set_ylabel('x north [m]')
        ax.set_title('Trajectory  (○=start, □=end)')
        ax.set_aspect('equal', adjustable='datalim')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=9)

        fig.suptitle(f'Drone {i} summary', fontsize=13)
        fig.tight_layout()
        out = out_dir / f'drone_{i}_summary.png'
        fig.savefig(out, dpi=150)
        plt.close(fig)
        print(f'[saved] {out}')


def export_csv(drones, num_drones, leader_df, out_dir):
    """Write per-drone CSV (merged pos + att on common time grid) and leader CSV."""
    for i in range(num_drones):
        pos = drones[i]['pos']
        att = drones[i]['att']
        if pos.empty and att.empty:
            continue
        # Use position timestamps as the master clock; resample/merge attitude
        # by nearest match.
        if not pos.empty and not att.empty:
            pos_s = pos.sort_values('t').reset_index(drop=True)
            att_s = att.sort_values('t').reset_index(drop=True)
            merged = pd.merge_asof(
                pos_s, att_s[['t', 'roll', 'pitch', 'yaw']],
                on='t', direction='nearest',
                tolerance=0.1,
            )
            cols = ['t', 'x', 'y', 'z', 'vx', 'vy', 'vz',
                    'roll', 'pitch', 'yaw', 'heading']
            cols = [c for c in cols if c in merged.columns]
            merged = merged[cols]
        elif not pos.empty:
            merged = pos[['t', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'heading']]
        else:
            merged = att[['t', 'roll', 'pitch', 'yaw']]
        out = out_dir / f'drone_{i}.csv'
        merged.to_csv(out, index=False)
        print(f'[saved] {out}')

    if leader_df is not None and not leader_df.empty:
        out = out_dir / 'leader.csv'
        cols = ['t', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'yaw']
        cols = [c for c in cols if c in leader_df.columns]
        leader_df[cols].to_csv(out, index=False)
        print(f'[saved] {out}')


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------

def detect_num_drones(bag_dir):
    """Peek at topics and infer number of drones from /px4_N/ namespaces."""
    storage = StorageOptions(uri=str(bag_dir), storage_id='sqlite3')
    converter = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr',
    )
    reader = SequentialReader()
    reader.open(storage, converter)
    topics = [t.name for t in reader.get_all_topics_and_types()]
    has_zero = any(t == '/fmu/out/vehicle_local_position' for t in topics)
    indices = set()
    if has_zero:
        indices.add(0)
    import re
    pat = re.compile(r'^/px4_(\d+)/fmu/out/vehicle_local_position$')
    for t in topics:
        m = pat.match(t)
        if m:
            indices.add(int(m.group(1)))
    if not indices:
        return 0
    return max(indices) + 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', help='ROS2 bag directory (the folder, not the .db3)')
    parser.add_argument('-n', '--num-drones', type=int, default=None,
                        help='Number of drones (auto-detected if omitted)')
    parser.add_argument('-o', '--out', default=None,
                        help='Output dir for plots+CSV (default: <bag>/plots)')
    args = parser.parse_args()

    bag_dir = Path(args.bag).expanduser().resolve()
    if not bag_dir.is_dir():
        print(f'[error] bag dir not found: {bag_dir}')
        sys.exit(1)

    num_drones = args.num_drones
    if num_drones is None:
        num_drones = detect_num_drones(bag_dir)
        print(f'[info] auto-detected num_drones = {num_drones}')
    if num_drones <= 0:
        print('[error] could not detect any drones in bag')
        sys.exit(1)

    out_dir = Path(args.out) if args.out else (bag_dir / 'plots')
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f'[info] output dir: {out_dir}')

    drones, leader_df, num_drones = read_bag(bag_dir, num_drones)

    # Overlay plots
    plot_position_overlay(drones, num_drones, out_dir)
    plot_velocity_overlay(drones, num_drones, out_dir)
    plot_attitude_overlay(drones, num_drones, out_dir)
    plot_trajectory_overlay(drones, num_drones, leader_df, out_dir)

    # Per-drone plots
    plot_per_drone(drones, num_drones, leader_df, out_dir)

    # CSV exports
    export_csv(drones, num_drones, leader_df, out_dir)

    print('[done]')


if __name__ == '__main__':
    main()