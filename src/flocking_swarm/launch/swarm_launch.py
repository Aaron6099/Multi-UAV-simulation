#!/usr/bin/env python3
"""
Launches the full flocking swarm — v3 (paper-strict).

Implements J.Yao's formula (19): each drone has an explicit formation offset
r_i0 relative to the leader, and uses a fixed sparse neighbour topology.

  - 1 virtual_leader_node
  - 9 flock_controller_node (one per drone, drone_id=0..8)
  - 1 arming_node

Topology (fixed, sparse, matches paper-style Laplacian design):
        北
   7  ──  3  ──  5
   │     │     │
   2  ──  0  ──  1     东
   │     │     │
   8  ──  4  ──  6
        南

Each drone subscribes to: virtual leader + its 4-connected neighbours.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


# =====================================================================
# Configuration
# =====================================================================

NUM_DRONES = 9

# Spawn positions in WORLD-NED (x=north, y=east, z=down).
# Drone N spawns at this location in Gazebo; controller adds this back to
# PX4's local position (which is relative to its own spawn).
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

# Formation offsets r_i0 in WORLD-NED, i.e. each drone's desired position
# RELATIVE TO THE LEADER. For the simplest case (formation = spawn shape just
# translating wherever the leader goes), set this equal to BIRTH_POSITIONS_FLAT.
# In other words: drone i wants to be at (leader_pos + r_i0).
FORMATION_OFFSETS_FLAT = BIRTH_POSITIONS_FLAT  # same shape as spawn = "fly the grid"

# Neighbour topology: who does each drone subscribe to (besides leader)?
# Sparse 4-connected grid. The leader (j=0 in paper) is implicitly added by
# the controller, so do NOT list it here.
NEIGHBOURS_PER_DRONE = {
    0: [1, 2, 3, 4],       # center: 4 cardinal neighbours
    1: [0, 5, 6],          # east:   center, NE, SE
    2: [0, 7, 8],          # west:   center, NW, SW
    3: [0, 5, 7],          # north:  center, NE, NW
    4: [0, 6, 8],          # south:  center, SE, SW
    5: [1, 3],             # NE:     east, north
    6: [1, 4],             # SE:     east, south
    7: [2, 3],             # NW:     west, north
    8: [2, 4],             # SW:     west, south
}

# Common controller params (LMI-derived gains — DO NOT change without re-solving LMI)
COMMON_PARAMS = {
    'num_drones': NUM_DRONES,
    'birth_positions_flat': BIRTH_POSITIONS_FLAT,
    'formation_offsets_flat': FORMATION_OFFSETS_FLAT,
    'c_pos': 0.2307,                 # Kp from LMI
    'c_vel': 0.7221,                 # Kv from LMI
    'filter_alpha': 0.5,             # low-pass filter coefficient
    'target_alt': -5.0,              # NED z (negative = up), so -5 = 5m above ground
    'z_kp': 0.8,                     # altitude P-gain (independent of LMI)
    'max_speed': 5.0,
    'max_climb': 1.5,
    'max_accel': 5.0,
    'control_hz': 50.0,              # paper uses 50 Hz (20ms loop)
    'neighbour_timeout': 1.0,
    'startup_zero_vel_frames': 30,
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
    'setup_seconds': 12.0,           # wait 12s after launch for controllers to stream
    'arm_interval': 0.5,
}


def generate_launch_description():
    nodes = []

    # 1. Virtual leader
    nodes.append(Node(
        package='flocking_swarm',
        executable='virtual_leader_node',
        name='virtual_leader',
        output='screen',
        parameters=[LEADER_PARAMS],
    ))

    # 2. 9 flock controllers, each with its own neighbour list
    for i in range(NUM_DRONES):
        params = dict(COMMON_PARAMS)
        params['drone_id'] = i
        params['neighbours'] = NEIGHBOURS_PER_DRONE[i]
        nodes.append(Node(
            package='flocking_swarm',
            executable='flock_controller_node',
            name=f'flock_controller_{i}',
            output='screen',
            parameters=[params],
        ))

    # 3. Arming node
    nodes.append(Node(
        package='flocking_swarm',
        executable='arming_node',
        name='arming_node',
        output='screen',
        parameters=[ARMING_PARAMS],
    ))

    return LaunchDescription(nodes)