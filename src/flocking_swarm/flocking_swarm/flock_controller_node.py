#!/usr/bin/env python3
"""
Distributed flocking controller node

This implements the second-order distributed consensus controller from
J.Yao's  "ROS2+PX4 无人机编队仿真（五）二阶分布式控制集群系统",
specifically formula (19) which adds explicit formation offsets r_ij:

    a_i = Kp * Σ_{j∈Ni} [(p_j - p_i) + r_ij] + Kv * Σ_{j∈Ni} (v_j - v_i)
                     ↑↑↑↑↑↑↑
              this offset is what was MISSING in v2

Where:
  - p_i, v_i: this drone's position/velocity in WORLD frame (NED)
  - p_j, v_j: neighbour j's position/velocity (also includes virtual leader as j=0)
  - r_ij = r_i0 - r_j0: desired relative offset of drone i from drone j
  - r_i0: desired offset of drone i from the leader (configured per-drone)

After computing virtual acceleration a, integrate to velocity and low-pass filter:
    v_des_raw = v_des_prev + a * dt
    v_cmd     = α * v_des_raw + (1-α) * v_des_prev

Gains from LMI solution (paper):  Kp = 0.6983,  Kv = 2.1929,  α = 0.5

The fixed neighbour topology is now explicitly configured rather than computed
from a distance threshold — this matches the paper's Laplacian-matrix design
and makes the LMI stability proof valid.

Z (altitude) is held with a simple P-controller, separate from the consensus
algorithm (paper sets a_z = 0 and lets PX4's own altitude controller hold;
here we add a P term so all drones converge to the same altitude).
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    VehicleLocalPosition,
    VehicleAttitude,
    OffboardControlMode,
    TrajectorySetpoint,
)
from std_msgs.msg import Float32MultiArray


def make_px4_qos():
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def quaternion_to_yaw(q):
    w, x, y, z = q[0], q[1], q[2], q[3]
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def safe_finite(x, default=0.0):
    return float(x) if math.isfinite(x) else float(default)


def topic_for_drone(drone_id, suffix):
    """Per project convention: drone 0 has empty namespace, drone N>=1 has /px4_N."""
    if drone_id == 0:
        return f'/fmu/{suffix}'
    return f'/px4_{drone_id}/fmu/{suffix}'


class DroneState:
    """Latest known WORLD-NED state of one drone."""
    def __init__(self):
        self.received = False
        self.last_stamp = 0.0
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.yaw = 0.0


class FlockControllerNode(Node):
    def __init__(self):
        super().__init__('flock_controller_node')

        # ---------- Parameters ----------
        self.declare_parameter('drone_id', 0)
        self.declare_parameter('num_drones', 9)

        # Birth positions (where each drone spawned in Gazebo, world NED)
        # Default: 3x3 grid spaced 3m
        default_births = [
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
        self.declare_parameter('birth_positions_flat', default_births)

        # Formation offsets r_i0: each drone's desired position relative to the leader
        # (in world NED frame). Default = same as birth positions (i.e. fly the
        # same formation as spawn, just translated to wherever the leader goes).
        self.declare_parameter('formation_offsets_flat', default_births)

        # Neighbour list for THIS drone (which other drones to subscribe to).
        # The virtual leader is ALWAYS implicitly in the neighbour set; this
        # parameter only lists OTHER followers.
        # Default: empty (filled in by launch file)
        self.declare_parameter('neighbours', [0])  # placeholder, must be set

        # Gains from LMI (paper)
        self.declare_parameter('c_pos', 0.6983)
        self.declare_parameter('c_vel', 2.1929)
        self.declare_parameter('filter_alpha', 0.5)

        # Z controller (separate from consensus)
        self.declare_parameter('target_alt', -5.0)
        self.declare_parameter('z_kp', 0.8)              # bumped from 0.5 → 0.8 for tighter alt hold
        self.declare_parameter('max_climb', 1.5)

        # Safety limits
        self.declare_parameter('max_speed', 5.0)
        self.declare_parameter('max_accel', 5.0)
        self.declare_parameter('control_hz', 50.0)        # paper uses 50 Hz (20 ms)
        self.declare_parameter('neighbour_timeout', 1.0)
        self.declare_parameter('startup_zero_vel_frames', 30)

        # ---- Read parameters ----
        self.drone_id = int(self.get_parameter('drone_id').value)
        self.num_drones = int(self.get_parameter('num_drones').value)

        births = list(self.get_parameter('birth_positions_flat').value)
        if len(births) != 3 * self.num_drones:
            raise RuntimeError(
                f'birth_positions_flat must have {3*self.num_drones} elements, got {len(births)}')
        self.birth_positions = np.array(births, dtype=float).reshape(self.num_drones, 3)

        offsets = list(self.get_parameter('formation_offsets_flat').value)
        if len(offsets) != 3 * self.num_drones:
            raise RuntimeError(
                f'formation_offsets_flat must have {3*self.num_drones} elements')
        self.formation_offsets = np.array(offsets, dtype=float).reshape(self.num_drones, 3)
        self.my_offset = self.formation_offsets[self.drone_id]  # r_i0 for THIS drone

        # Neighbour list (other followers I subscribe to; leader is implicit)
        neighbours_raw = list(self.get_parameter('neighbours').value)
        # Filter: drop self, drop -1 placeholder, drop out-of-range indices
        self.neighbours = sorted(set(
            int(j) for j in neighbours_raw
            if 0 <= int(j) < self.num_drones and int(j) != self.drone_id
        ))

        self.c_pos = float(self.get_parameter('c_pos').value)
        self.c_vel = float(self.get_parameter('c_vel').value)
        self.filter_alpha = float(self.get_parameter('filter_alpha').value)
        self.target_alt = float(self.get_parameter('target_alt').value)
        self.z_kp = float(self.get_parameter('z_kp').value)
        self.max_climb = float(self.get_parameter('max_climb').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_accel = float(self.get_parameter('max_accel').value)
        self.control_hz = float(self.get_parameter('control_hz').value)
        self.neighbour_timeout = float(self.get_parameter('neighbour_timeout').value)
        self.startup_zero_vel_frames = int(self.get_parameter('startup_zero_vel_frames').value)

        # ---------- State ----------
        self.drone_states = [DroneState() for _ in range(self.num_drones)]

        self.leader_received = False
        self.leader_pos = np.zeros(3)
        self.leader_vel = np.zeros(3)
        self.leader_yaw = 0.0

        self.attitude_yaw = 0.0
        self.attitude_received = False

        self.desired_vel = np.zeros(3)  # state of the integrator+filter

        self.last_control_time = self.get_clock().now()
        self._startup_counter = 0

        # ---------- QoS ----------
        qos = make_px4_qos()

        # ---------- Publishers ----------
        self.pub_offboard_mode = self.create_publisher(
            OffboardControlMode,
            topic_for_drone(self.drone_id, 'in/offboard_control_mode'),
            qos,
        )
        self.pub_setpoint = self.create_publisher(
            TrajectorySetpoint,
            topic_for_drone(self.drone_id, 'in/trajectory_setpoint'),
            qos,
        )

        # ---------- Subscribers ----------
        # Own attitude (for current yaw)
        self._att_sub = self.create_subscription(
            VehicleAttitude,
            topic_for_drone(self.drone_id, 'out/vehicle_attitude'),
            self.on_self_attitude,
            qos,
        )

        # Own position (always needed)
        self._own_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            topic_for_drone(self.drone_id, 'out/vehicle_local_position'),
            self._make_pos_callback(self.drone_id),
            qos,
        )

        # Subscribe to each NEIGHBOUR's position (NOT every drone — sparse topology)
        self._neighbour_pos_subs = []
        for j in self.neighbours:
            sub = self.create_subscription(
                VehicleLocalPosition,
                topic_for_drone(j, 'out/vehicle_local_position'),
                self._make_pos_callback(j),
                qos,
            )
            self._neighbour_pos_subs.append(sub)

        # Virtual leader state
        leader_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._leader_sub = self.create_subscription(
            Float32MultiArray, '/leader/state', self.on_leader_state, leader_qos
        )

        # ---------- Control timer ----------
        self.timer = self.create_timer(1.0 / self.control_hz, self.control_loop)

        self.get_logger().info(
            f'flock_controller v3 drone {self.drone_id} ready. '
            f'birth=({self.birth_positions[self.drone_id]}), '
            f'r_i0=({self.my_offset}), '
            f'neighbours={self.neighbours}, '
            f'Kp={self.c_pos}, Kv={self.c_vel}, alpha={self.filter_alpha}, '
            f'control_hz={self.control_hz}'
        )

    # ============================================================
    # Subscriber callbacks
    # ============================================================

    def _make_pos_callback(self, drone_idx):
        def cb(msg):
            now = self.get_clock().now().nanoseconds * 1e-9
            ds = self.drone_states[drone_idx]
            if not ds.received:
                self.get_logger().info(
                    f'first position from drone {drone_idx}: '
                    f'local=({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f})'
                )
            ds.received = True
            ds.last_stamp = now
            # Convert PX4 local frame (origin at spawn) to WORLD frame by adding birth offset
            ds.pos = np.array([msg.x, msg.y, msg.z]) + self.birth_positions[drone_idx]
            ds.vel = np.array([msg.vx, msg.vy, msg.vz])
            ds.yaw = float(msg.heading) if math.isfinite(msg.heading) else 0.0
        return cb

    def on_self_attitude(self, msg):
        self.attitude_yaw = quaternion_to_yaw(msg.q)
        self.attitude_received = True

    def on_leader_state(self, msg):
        if len(msg.data) < 8:
            return
        self.leader_received = True
        self.leader_pos = np.array([msg.data[1], msg.data[2], msg.data[3]])
        self.leader_vel = np.array([msg.data[4], msg.data[5], msg.data[6]])
        self.leader_yaw = float(msg.data[7]) if math.isfinite(msg.data[7]) else 0.0

    # ============================================================
    # Control loop
    # ============================================================

    def control_loop(self):
        # 1. Always stream OffboardControlMode
        self.publish_offboard_mode()

        # 2. dt for integrator
        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds * 1e-9
        self.last_control_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / self.control_hz

        # 3. Startup phase: zero-velocity stream so PX4 sees stable setpoints before arming
        if self._startup_counter < self.startup_zero_vel_frames:
            self._startup_counter += 1
            self.publish_velocity_setpoint([0.0, 0.0, 0.0], 0.0)
            return

        self_ds = self.drone_states[self.drone_id]
        if not self_ds.received:
            self.publish_velocity_setpoint([0.0, 0.0, 0.0], 0.0)
            return

        # 4. Without leader, hold position at target altitude
        if not self.leader_received:
            z_err = self.target_alt - self_ds.pos[2]
            vz = max(-self.max_climb, min(self.max_climb, self.z_kp * z_err))
            self.publish_velocity_setpoint([0.0, 0.0, vz], 0.0)
            return

        # 5. Main: paper formula (19) — consensus with explicit formation offsets
        a_xy = self.compute_consensus_acceleration_xy()

        # NaN guard
        if not np.all(np.isfinite(a_xy)):
            self.get_logger().warn(f'NaN in acceleration {a_xy}, falling back to zero')
            a_xy = np.zeros(2)

        # Acceleration magnitude limit
        a_mag = float(np.linalg.norm(a_xy))
        if a_mag > self.max_accel:
            a_xy *= self.max_accel / a_mag

        # 6. Integrate to velocity (formula 22)
        v_xy_raw = self.desired_vel[:2] + a_xy * dt

        # 7. Low-pass filter (formula 23)
        self.desired_vel[:2] = (
            self.filter_alpha * v_xy_raw +
            (1.0 - self.filter_alpha) * self.desired_vel[:2]
        )

        # 8. Speed limit (horizontal)
        v_xy_norm = float(np.linalg.norm(self.desired_vel[:2]))
        if v_xy_norm > self.max_speed:
            self.desired_vel[:2] *= self.max_speed / v_xy_norm

        # 9. Z: simple P controller (separate from consensus)
        z_err = self.target_alt - self_ds.pos[2]
        self.desired_vel[2] = max(-self.max_climb, min(self.max_climb, self.z_kp * z_err))

        # 10. NaN guard on output
        if not np.all(np.isfinite(self.desired_vel)):
            self.get_logger().warn('NaN in desired_vel, resetting')
            self.desired_vel = np.zeros(3)

        # 11. Yaw: follow direction of motion when moving, else align with leader
        if v_xy_norm > 0.5:
            yaw_sp = math.atan2(self.desired_vel[1], self.desired_vel[0])
        else:
            yaw_sp = self.leader_yaw

        self.publish_velocity_setpoint(self.desired_vel.tolist(), yaw_sp)

    # ============================================================
    # Consensus acceleration (paper formula 19, xy only)
    # ============================================================

    def compute_consensus_acceleration_xy(self):
        """Compute virtual acceleration in the xy plane:

            a = Kp * Σ [(p_j - p_i) + r_ij]  +  Kv * Σ (v_j - v_i)

        Sum runs over the leader (j=0 implicit) AND each configured neighbour.
        r_ij = r_i0 - r_j0  (where r_i0 is the configured offset from leader for drone i)
        For the leader itself, r_j0 = 0 by definition, so r_ij_leader = r_i0.
        """
        self_ds = self.drone_states[self.drone_id]
        my_pos = self_ds.pos[:2]
        my_vel = self_ds.vel[:2]
        my_offset = self.my_offset[:2]      # r_i0 (xy)

        now = self.get_clock().now().nanoseconds * 1e-9

        sum_dp = np.zeros(2)
        sum_dv = np.zeros(2)

        # ------ Leader contribution (j=0 in the paper) ------
        # r_ij = r_i0 - 0 = r_i0
        # term = (p_leader - p_self) + r_i0
        # Meaning: "I want my position to be (leader_pos + my_offset) ahead of where I am now"
        leader_term_pos = (self.leader_pos[:2] - my_pos) + my_offset
        leader_term_vel = (self.leader_vel[:2] - my_vel)
        sum_dp += leader_term_pos
        sum_dv += leader_term_vel

        # ------ Configured neighbour contributions ------
        for j in self.neighbours:
            ds = self.drone_states[j]
            if not ds.received:
                continue
            if (now - ds.last_stamp) > self.neighbour_timeout:
                continue
            r_ij = my_offset - self.formation_offsets[j][:2]   # = r_i0 - r_j0
            # term = (p_j - p_i) + r_ij
            sum_dp += (ds.pos[:2] - my_pos) + r_ij
            sum_dv += (ds.vel[:2] - my_vel)

        a = self.c_pos * sum_dp + self.c_vel * sum_dv
        return a

    # ============================================================
    # PX4 publishers
    # ============================================================

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_offboard_mode.publish(msg)

    def publish_velocity_setpoint(self, vel_world_ned, yaw):
        vx = safe_finite(vel_world_ned[0], 0.0)
        vy = safe_finite(vel_world_ned[1], 0.0)
        vz = safe_finite(vel_world_ned[2], 0.0)
        yaw_safe = safe_finite(yaw, 0.0)

        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [vx, vy, vz]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = yaw_safe
        msg.yawspeed = float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_setpoint.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FlockControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()