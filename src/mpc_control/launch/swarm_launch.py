#!/usr/bin/env python3
"""
Launches the MPC swarm with selectable formation.

Usage:
  ros2 launch mpc_control swarm_launch.py                  # default: cross5
  ros2 launch mpc_control swarm_launch.py formation:=line2 # 2-drone line
  ros2 launch mpc_control swarm_launch.py formation:=cross5

Formations:
  line2  — 2 drones, 3m apart (east-west)
  cross5 — 5 drones, cross topology, 3m spacing
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# =====================================================================
# Formation definitions
# =====================================================================

FORMATIONS = {
    'line2': {
        'birth': [
            0.0,  0.0, 0.0,   # 0: center
            0.0,  3.0, 0.0,   # 1: east
        ],
        'neighbours': {
            0: [1],
            1: [0],
        },
    },
    'cross5': {
        'birth': [
            0.0,  0.0, 0.0,   # 0: center
            0.0,  3.0, 0.0,   # 1: east
            0.0, -3.0, 0.0,   # 2: west
            3.0,  0.0, 0.0,   # 3: north
           -3.0,  0.0, 0.0,   # 4: south
        ],
        'neighbours': {
            0: [1, 2, 3, 4],
            1: [0],
            2: [0],
            3: [0],
            4: [0],
        },
    },
}


def generate_launch_description():
    formation_arg = DeclareLaunchArgument(
        'formation', default_value='cross5',
        description='Formation type: line2, cross5',
    )
    leader_speed_arg = DeclareLaunchArgument(
        'leader_speed', default_value='0.0',
        description='Leader speed (m/s). 0=hover, >0=move along waypoints',
    )

    # We resolve the formation at launch-gen time via a shim:
    # ros2 launch passes 'formation:=line2' which DeclareLaunchArgument
    # stores, but LaunchConfiguration is only resolved at runtime.
    # Since we need NUM_DRONES at parse time for the for-loop, we
    # parse sys.argv directly as a fallback.
    import sys
    resolved = 'cross5'
    ldr_speed = 0.0
    for a in sys.argv:
        if 'formation:=' in a:
            resolved = a.split(':=')[-1]
        if 'leader_speed:=' in a:
            ldr_speed = float(a.split(':=')[-1])

    if resolved not in FORMATIONS:
        raise ValueError(
            f'Unknown formation "{resolved}". '
            f'Choose from: {list(FORMATIONS.keys())}')

    cfg = FORMATIONS[resolved]
    num_drones = len(cfg['birth']) // 3
    births = cfg['birth']
    neighbours = cfg['neighbours']

    common = {
        'num_drones': num_drones,
        'birth_positions_flat': births,
        'formation_offsets_flat': births,
        'target_alt': -5.0,
        'max_speed': 5.0,
        'max_climb': 1.5,
        'max_accel': 5.0,
        'control_hz': 50.0,
        'neighbour_timeout': 1.0,
        'startup_zero_vel_frames': 30,
        'mpc_horizon': 20,
        'mpc_dt':      0.05,
        'q_pos':       8.0,
        'q_vel':       2.0,
        'r_acc':       0.05,
        'q_pos_terminal_scale': 2.0,
        'd_safe':      1.5,
        'w_collision': 500.0,
        'w_formation': 0.5,
        'acados_build_dir': '/tmp/acados_di_mpc',
    }

    leader = {
        'speed': ldr_speed,
        'altitude': -5.0,
        'publish_hz': 50.0,
        'waypoints_flat': [0.0, 0.0,  0.0, 50.0,  50.0, 50.0,  50.0, 0.0],
    }

    arming = {
        'num_drones': num_drones,
        'setup_seconds': 15.0 if num_drones <= 2 else 20.0,
        'arm_interval': 0.5,
    }

    nodes = [formation_arg, leader_speed_arg]

    nodes.append(Node(
        package='mpc_control',
        executable='virtual_leader_node',
        name='virtual_leader',
        output='screen',
        parameters=[leader],
    ))

    for i in range(num_drones):
        params = dict(common)
        params['drone_id'] = i
        params['neighbours'] = neighbours[i]
        nodes.append(Node(
            package='mpc_control',
            executable='mpc_node',
            name=f'mpc_controller_{i}',
            output='screen',
            parameters=[params],
        ))

    nodes.append(Node(
        package='mpc_control',
        executable='arming_node',
        name='arming_node',
        output='screen',
        parameters=[arming],
    ))

    return LaunchDescription(nodes)