#!/usr/bin/env python3
"""
Launches the full MPC swarm — 9-drone DMPC version.

Uses acados-based per-vehicle MPC 

Topology:
        北
   7  ──  3  ──  5
   │     │     │
   2  ──  0  ──  1     东
   │     │     │
   8  ──  4  ──  6
        南
"""

from launch import LaunchDescription
from launch_ros.actions import Node


# =====================================================================
# Configuration  ==  IDENTICAL to v3, except LMI params replaced with MPC
# =====================================================================

NUM_DRONES = 9

BIRTH_POSITIONS_FLAT = [
    0.0,  0.0, 0.0,   # 0: center
    0.0,  3.0, 0.0,   # 1: east
    0.0, -3.0, 0.0,   # 2: west
    3.0,  0.0, 0.0,   # 3: north
   -3.0,  0.0, 0.0,   # 4: south
    3.0,  3.0, 0.0,   # 5: NE
    3.0, -3.0, 0.0,   # 6: SE
   -3.0,  3.0, 0.0,   # 7: NW
   -3.0, -3.0, 0.0,   # 8: SW
]

# Same shape as spawn = formation translates to wherever the leader goes.
FORMATION_OFFSETS_FLAT = BIRTH_POSITIONS_FLAT

# Sparse 4-connected grid — leader is implicit (NOT listed here).
NEIGHBOURS_PER_DRONE = {
    0: [1, 2, 3, 4],
    1: [0, 5, 6],
    2: [0, 7, 8],
    3: [0, 5, 7],
    4: [0, 6, 8],
    5: [1, 3],
    6: [1, 4],
    7: [2, 3],
    8: [2, 4],
}

# ----------------------------------------------------------------------
# Per-drone params for the MPC controller.
# ----------------------------------------------------------------------
COMMON_PARAMS = {
    # ---- topology / geometry (KEPT identical to v3) ----
    'num_drones': NUM_DRONES,
    'birth_positions_flat': BIRTH_POSITIONS_FLAT,
    'formation_offsets_flat': FORMATION_OFFSETS_FLAT,

    # ---- altitude target (KEPT) ----
    'target_alt': -5.0,                  # NED z, -5 = 5m above ground

    # ---- safety / speed limits (KEPT — they are MPC hard constraints now) ----
    'max_speed': 5.0,                    # |v_xy| upper bound, m/s
    'max_climb': 1.5,                    # |v_z|  upper bound, m/s
    'max_accel': 5.0,                    # |u|    upper bound, m/s^2

    # ---- loop timing (KEPT) ----
    'control_hz': 50.0,                  # 50 Hz outer loop, same as v3
    'neighbour_timeout': 1.0,
    'startup_zero_vel_frames': 30,

    # ---- MPC-specific (NEW) ----
    'mpc_horizon': 20,                   # N stages
    'mpc_dt':      0.05,                 # stage length s; 20*0.05 = 1.0s look-ahead
    'q_pos':       4.0,                  # tracking position weight
    'q_vel':       1.0,                  # tracking velocity weight
    'r_acc':       0.1,                  # action smoothness weight
    'q_pos_terminal_scale': 10.0,        # terminal vs running pos weight ratio
    'd_safe':      1.5,                  # min inter-agent distance, m
    'w_collision': 200.0,                # soft-penalty weight for d_safe breach
    'acados_build_dir': '/tmp/acados_di_mpc',
}

LEADER_PARAMS = {
    'speed': 2.0,
    'altitude': -5.0,
    'publish_hz': 50.0,
    # Rectangle 50m x 50m starting at origin
    'waypoints_flat': [0.0, 0.0,  0.0, 50.0,  50.0, 50.0,  50.0, 0.0],
}

ARMING_PARAMS = {
    'num_drones': NUM_DRONES,
    'setup_seconds': 25.0,               # bumped from 12 -> 25 because acados
                                         # JIT-compiles the OCP on first call
                                         # for each drone (~1-2s per drone)
    'arm_interval': 0.5,
}


def generate_launch_description():
    nodes = []

    # 1. Virtual leader (UNCHANGED)
    nodes.append(Node(
        package='mpc_control',
        executable='virtual_leader_node',
        name='virtual_leader',
        output='screen',
        parameters=[LEADER_PARAMS],
    ))

    # 2. 9 MPC controllers (was: flock_controller_node)
    for i in range(NUM_DRONES):
        params = dict(COMMON_PARAMS)
        params['drone_id'] = i
        params['neighbours'] = NEIGHBOURS_PER_DRONE[i]
        nodes.append(Node(
            package='mpc_control',
            executable='mpc_node',
            name=f'mpc_controller_{i}',
            output='screen',
            parameters=[params],
        ))

    # 3. Arming node (UNCHANGED, except setup_seconds bumped above)
    nodes.append(Node(
        package='mpc_control',
        executable='arming_node',
        name='arming_node',
        output='screen',
        parameters=[ARMING_PARAMS],
    ))

    return LaunchDescription(nodes)