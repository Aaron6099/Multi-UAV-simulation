#!/usr/bin/env python3
"""
MPC controller node — drop-in replacement for flock_controller_node.py (v3).

KEY PRINCIPLE
=============
Same I/O contract as flock_controller_node v3:
  Subs : <ns>/fmu/out/vehicle_local_position
         <ns>/fmu/out/vehicle_attitude
         /leader/state                  (Float32MultiArray, 8 dim)
         <neighbour_ns>/fmu/out/vehicle_local_position  (sparse topology)
         <neighbour_ns>/mpc/predicted_trajectory        (NEW, optional)
  Pubs : <ns>/fmu/in/offboard_control_mode
         <ns>/fmu/in/trajectory_setpoint            (velocity mode, same as v3)
         <ns>/mpc/predicted_trajectory              (NEW, broadcast for peers)

Same parameters as v3 are KEPT (birth_positions_flat, formation_offsets_flat,
neighbours, target_alt, max_speed, max_climb, max_accel, control_hz,
neighbour_timeout, startup_zero_vel_frames). The LMI gains (c_pos / c_vel /
filter_alpha) are dropped — replaced by MPC weights.

CONTROLLER MODEL
================
Per-vehicle linear double-integrator MPC, solved with acados:
    state x = [px, py, pz, vx, vy, vz]    (6 dim, world NED)
    input u = [ax, ay, az]                (3 dim, world NED)

    x_{k+1} = A x_k + B u_k     (zero-order hold, dt = mpc_dt)

    minimise   Σ_{k=0..N-1} ‖x_k - x_ref[k]‖²_Q + ‖u_k‖²_R
             + ‖x_N - x_ref[N]‖²_QN
             + Σ_neighbours soft-collision penalty

    s.t.       |v_xy| ≤ max_speed, |v_z| ≤ max_climb, |u| ≤ max_accel

Reference at stage k:
    x_ref[k].pos = leader_pos + leader_vel * (k*dt) + my_offset
    x_ref[k].vel = leader_vel
The first row is the closest-future setpoint; we publish v_pred[1] (one step
ahead of current state) to PX4, exactly the way v3 did with v_des.

DMPC COUPLING
=============
Each vehicle broadcasts its predicted trajectory on /<ns>/mpc/predicted_trajectory
as a Float64MultiArray of shape (N+1, 3) row-major. Peers parse this and feed
the predicted positions into per-stage parameters of the OCP, where a smooth
penalty kicks in when |p_self - p_neighbour| < d_safe.

If a peer hasn't broadcast yet (e.g. before its MPC is up), we fall back to
its CURRENT position from vehicle_local_position, replicated across all stages.
This means avoidance starts working immediately, before peers' predictions
come online.
"""

import math
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)

from std_msgs.msg import Float32MultiArray, Float64MultiArray, MultiArrayDimension
from px4_msgs.msg import (
    VehicleLocalPosition,
    VehicleAttitude,
    OffboardControlMode,
    TrajectorySetpoint,
)


# ------------------------------------------------------------------ helpers
def make_px4_qos():
    """QoS matching PX4 v1.14 uXRCE-DDS publishers."""
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
    """Project naming rule: drone 0 -> /fmu/...; drone i -> /px4_i/fmu/..."""
    if drone_id == 0:
        return f'/fmu/{suffix}'
    return f'/px4_{drone_id}/fmu/{suffix}'


def mpc_topic_for_drone(drone_id, suffix='predicted_trajectory'):
    """Per-vehicle MPC topic, follows the same naming rule."""
    if drone_id == 0:
        return f'/mpc/{suffix}'
    return f'/px4_{drone_id}/mpc/{suffix}'


# ------------------------------------------------------------------ MPC core
class DoubleIntegratorMPC:
    """
    Linear double-integrator MPC built on acados, with per-stage parameters
    encoding neighbour positions for soft collision avoidance.

    Built once at startup; subsequent solves are fast (~0.3-0.5 ms typical).
    """

    def __init__(
        self,
        N=20,
        dt=0.05,
        max_speed=5.0,
        max_climb=1.5,
        max_accel=5.0,
        max_neighbours=4,
        d_safe=1.5,
        w_collision=200.0,
        q_pos=4.0,
        q_vel=1.0,
        r_acc=0.1,
        q_pos_terminal_scale=10.0,
        build_dir='/tmp/acados_di_mpc',
        instance_id=0,
    ):
        self.N = N
        self.dt = dt
        self.max_speed = max_speed
        self.max_climb = max_climb
        self.max_accel = max_accel
        self.max_neighbours = max(1, int(max_neighbours))
        self.d_safe = d_safe
        self.w_collision = w_collision

        self.q_pos = q_pos
        self.q_vel = q_vel
        self.r_acc = r_acc
        self.q_pos_terminal_scale = q_pos_terminal_scale

        self._build_dir = f'{build_dir}_v{instance_id}'
        os.makedirs(self._build_dir, exist_ok=True)
        self._instance_id = instance_id

        self.solver = None
        self._setup_ocp()

    # -----------------------------------------------------------------
    def _setup_ocp(self):
        import casadi as ca
        from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

        # ---- symbolic state / input ----
        nx, nu = 6, 3
        x = ca.SX.sym('x', nx)        # [px, py, pz, vx, vy, vz]
        u = ca.SX.sym('u', nu)        # [ax, ay, az]

        # Per-stage parameters: M neighbour positions (3*M) + active flags (M)
        M = self.max_neighbours
        p_n   = ca.SX.sym('p_n', 3 * M)
        p_act = ca.SX.sym('p_act', M)
        p_full = ca.vertcat(p_n, p_act)

        # ---- continuous dynamics: \dot x = [v; u] ----
        f_expl = ca.vertcat(x[3], x[4], x[5], u[0], u[1], u[2])

        model = AcadosModel()
        model.name = f'di_mpc_v{self._instance_id}'
        model.x = x
        model.u = u
        model.p = p_full
        model.f_expl_expr = f_expl
        xdot = ca.SX.sym('xdot', nx)
        model.xdot = xdot
        model.f_impl_expr = xdot - f_expl

        # ---- OCP ----
        ocp = AcadosOcp()
        ocp.model = model
        ocp.dims.N = self.N

        # Cost: NONLINEAR_LS so the soft collision residual fits naturally
        # y = [pos(3), vel(3), acc(3), collision_residual(1)]   length 10
        soft_terms = []
        for i in range(M):
            ni = p_n[3*i : 3*(i+1)]
            d2 = ca.sumsqr(x[0:3] - ni)
            breach = ca.fmax(0.0, self.d_safe**2 - d2)
            soft_terms.append(p_act[i] * breach)
        soft_sum = sum(soft_terms) if soft_terms else ca.SX(0.0)
        coll_residual = ca.sqrt(self.w_collision) * ca.sqrt(soft_sum + 1e-9)

        y_expr = ca.vertcat(x[0:3], x[3:6], u, coll_residual)
        y_expr_e = ca.vertcat(x[0:3], x[3:6])

        ocp.cost.cost_type = 'NONLINEAR_LS'
        ocp.cost.cost_type_e = 'NONLINEAR_LS'
        ocp.model.cost_y_expr = y_expr
        ocp.model.cost_y_expr_e = y_expr_e

        # Weights
        Q = np.diag([
            self.q_pos, self.q_pos, self.q_pos,        # pos
            self.q_vel, self.q_vel, self.q_vel,        # vel
            self.r_acc, self.r_acc, self.r_acc,        # acc input
            1.0,                                        # collision residual
        ])
        Qe = np.diag([
            self.q_pos * self.q_pos_terminal_scale,
            self.q_pos * self.q_pos_terminal_scale,
            self.q_pos * self.q_pos_terminal_scale,
            self.q_vel, self.q_vel, self.q_vel,
        ])
        ocp.cost.W = Q
        ocp.cost.W_e = Qe
        ocp.cost.yref = np.zeros(10)
        ocp.cost.yref_e = np.zeros(6)

        # ---- input box constraints ----
        ocp.constraints.lbu = np.array([-self.max_accel] * 3)
        ocp.constraints.ubu = np.array([+self.max_accel] * 3)
        ocp.constraints.idxbu = np.arange(nu)

        # ---- velocity box constraints (state) ----
        ocp.constraints.lbx = np.array([-self.max_speed, -self.max_speed, -self.max_climb])
        ocp.constraints.ubx = np.array([+self.max_speed, +self.max_speed, +self.max_climb])
        ocp.constraints.idxbx = np.array([3, 4, 5])

        # ---- initial state placeholder ----
        ocp.constraints.x0 = np.zeros(nx)

        # ---- per-stage parameter init ----
        ocp.parameter_values = np.zeros(p_full.shape[0])

        # ---- solver options ----
        ocp.solver_options.tf = self.N * self.dt
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_num_steps = 1
        ocp.solver_options.print_level = 0

        ocp.code_export_directory = os.path.join(self._build_dir, 'c_generated_code')
        json_file = os.path.join(self._build_dir, 'acados_ocp.json')
        self.solver = AcadosOcpSolver(ocp, json_file=json_file)

        self._nx, self._nu = nx, nu
        self._np = p_full.shape[0]

    # -----------------------------------------------------------------
    def solve(self, x0, x_ref, neighbour_traj=None):
        """
        Args:
            x0:             current state (6,)  in world NED
            x_ref:          (N+1, 6) per-stage reference [pos(3), vel(3)]
            neighbour_traj: (M, N+1, 3) predicted neighbour positions, or None.
                            Slots beyond actual M_real are zeroed (ignored).

        Returns:
            u0:     (3,)        first applied acceleration
            x_pred: (N+1, 6)    predicted state trajectory (broadcast to peers)
            info:   dict
        """
        N = self.N
        # set initial state
        self.solver.set(0, 'lbx', x0)
        self.solver.set(0, 'ubx', x0)

        # set references and neighbour parameters per stage
        for k in range(N):
            yref_k = np.concatenate([
                x_ref[k, 0:3],     # pos ref
                x_ref[k, 3:6],     # vel ref
                np.zeros(3),       # acc ref = 0 (cruise)
                np.array([0.0]),   # collision residual ref = 0
            ])
            self.solver.set(k, 'yref', yref_k)
            self.solver.set(k, 'p', self._pack_params(k, neighbour_traj))

        self.solver.set(N, 'yref', x_ref[N, 0:6])
        self.solver.set(N, 'p', self._pack_params(N, neighbour_traj))

        status = self.solver.solve()

        u0 = self.solver.get(0, 'u')
        x_pred = np.zeros((N + 1, self._nx))
        for k in range(N + 1):
            x_pred[k, :] = self.solver.get(k, 'x')

        info = {
            'status': int(status),
            'cost': float(self.solver.get_cost()),
            'solve_time_s': float(self.solver.get_stats('time_tot')),
        }
        return u0, x_pred, info

    # -----------------------------------------------------------------
    def _pack_params(self, k, neighbour_traj):
        M = self.max_neighbours
        p = np.zeros(3 * M + M)
        if neighbour_traj is None:
            return p

        nt = np.asarray(neighbour_traj)
        if nt.ndim != 3 or nt.shape[2] != 3:
            return p

        M_real = min(nt.shape[0], M)
        K = max(0, nt.shape[1] - 1)
        kc = min(k, K)
        for i in range(M_real):
            p[3*i : 3*(i+1)] = nt[i, kc, :]
            p[3*M + i]       = 1.0
        return p


# ------------------------------------------------------------------ state
class DroneState:
    def __init__(self):
        self.received = False
        self.last_stamp = 0.0
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.yaw = 0.0


# ------------------------------------------------------------------ node
class MpcControllerNode(Node):
    def __init__(self):
        super().__init__('mpc_controller_node')

        # =====================================================
        # Parameters — IDENTICAL set as flock_controller v3
        # plus a few MPC-specific ones at the end
        # =====================================================
        self.declare_parameter('drone_id', 0)
        self.declare_parameter('num_drones', 9)

        default_births = [
            0.0,  0.0, 0.0,
            0.0,  3.0, 0.0,
            0.0, -3.0, 0.0,
            3.0,  0.0, 0.0,
           -3.0,  0.0, 0.0,
            3.0,  3.0, 0.0,
            3.0, -3.0, 0.0,
           -3.0,  3.0, 0.0,
           -3.0, -3.0, 0.0,
        ]
        self.declare_parameter('birth_positions_flat', default_births)
        self.declare_parameter('formation_offsets_flat', default_births)
        self.declare_parameter('neighbours', [0])

        # Reused safety / loop-rate parameters
        self.declare_parameter('target_alt', -5.0)
        self.declare_parameter('max_speed', 5.0)
        self.declare_parameter('max_climb', 1.5)
        self.declare_parameter('max_accel', 5.0)
        self.declare_parameter('control_hz', 50.0)
        self.declare_parameter('neighbour_timeout', 1.0)
        self.declare_parameter('startup_zero_vel_frames', 30)

        # MPC-specific
        self.declare_parameter('mpc_horizon', 20)
        self.declare_parameter('mpc_dt', 0.05)            # 0.05*20 = 1.0s look-ahead
        self.declare_parameter('q_pos', 4.0)
        self.declare_parameter('q_vel', 1.0)
        self.declare_parameter('r_acc', 0.1)
        self.declare_parameter('q_pos_terminal_scale', 10.0)
        self.declare_parameter('d_safe', 1.5)             # min separation, m
        self.declare_parameter('w_collision', 200.0)
        self.declare_parameter('acados_build_dir', '/tmp/acados_di_mpc')

        # ---- Read parameters ----
        self.drone_id   = int(self.get_parameter('drone_id').value)
        self.num_drones = int(self.get_parameter('num_drones').value)

        births = list(self.get_parameter('birth_positions_flat').value)
        if len(births) != 3 * self.num_drones:
            raise RuntimeError(
                f'birth_positions_flat must have {3*self.num_drones} elements')
        self.birth_positions = np.array(births, dtype=float).reshape(self.num_drones, 3)

        offsets = list(self.get_parameter('formation_offsets_flat').value)
        if len(offsets) != 3 * self.num_drones:
            raise RuntimeError(
                f'formation_offsets_flat must have {3*self.num_drones} elements')
        self.formation_offsets = np.array(offsets, dtype=float).reshape(self.num_drones, 3)
        self.my_offset = self.formation_offsets[self.drone_id]

        neighbours_raw = list(self.get_parameter('neighbours').value)
        self.neighbours = sorted(set(
            int(j) for j in neighbours_raw
            if 0 <= int(j) < self.num_drones and int(j) != self.drone_id
        ))

        self.target_alt = float(self.get_parameter('target_alt').value)
        self.max_speed  = float(self.get_parameter('max_speed').value)
        self.max_climb  = float(self.get_parameter('max_climb').value)
        self.max_accel  = float(self.get_parameter('max_accel').value)
        self.control_hz = float(self.get_parameter('control_hz').value)
        self.neighbour_timeout = float(self.get_parameter('neighbour_timeout').value)
        self.startup_zero_vel_frames = int(self.get_parameter('startup_zero_vel_frames').value)

        N        = int(self.get_parameter('mpc_horizon').value)
        mpc_dt   = float(self.get_parameter('mpc_dt').value)
        q_pos    = float(self.get_parameter('q_pos').value)
        q_vel    = float(self.get_parameter('q_vel').value)
        r_acc    = float(self.get_parameter('r_acc').value)
        q_term_s = float(self.get_parameter('q_pos_terminal_scale').value)
        d_safe   = float(self.get_parameter('d_safe').value)
        w_coll   = float(self.get_parameter('w_collision').value)
        build_dir = str(self.get_parameter('acados_build_dir').value)

        # =====================================================
        # State
        # =====================================================
        self.drone_states = [DroneState() for _ in range(self.num_drones)]

        self.leader_received = False
        self.leader_pos = np.zeros(3)
        self.leader_vel = np.zeros(3)
        self.leader_yaw = 0.0

        self.attitude_yaw = 0.0
        self.attitude_received = False

        self.last_control_time = self.get_clock().now()
        self._startup_counter = 0

        # neighbour predicted-trajectory cache: dict[neighbour_id] -> (N+1, 3)
        self.peer_predictions = {}

        # =====================================================
        # MPC
        # =====================================================
        self.N = N
        self.mpc_dt = mpc_dt

        self.get_logger().info(
            f'building acados OCP for drone {self.drone_id} (N={N}, dt={mpc_dt})...'
        )
        self.mpc = DoubleIntegratorMPC(
            N=N, dt=mpc_dt,
            max_speed=self.max_speed,
            max_climb=self.max_climb,
            max_accel=self.max_accel,
            max_neighbours=max(1, len(self.neighbours)),
            d_safe=d_safe,
            w_collision=w_coll,
            q_pos=q_pos, q_vel=q_vel, r_acc=r_acc,
            q_pos_terminal_scale=q_term_s,
            build_dir=build_dir,
            instance_id=self.drone_id,
        )
        self.get_logger().info('acados OCP ready.')

        # =====================================================
        # ROS 2 IO — IDENTICAL TOPICS to v3
        # =====================================================
        qos = make_px4_qos()

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

        # Self attitude
        self.create_subscription(
            VehicleAttitude,
            topic_for_drone(self.drone_id, 'out/vehicle_attitude'),
            self.on_self_attitude,
            qos,
        )

        # Self position
        self.create_subscription(
            VehicleLocalPosition,
            topic_for_drone(self.drone_id, 'out/vehicle_local_position'),
            self._make_pos_callback(self.drone_id),
            qos,
        )

        # Neighbour positions
        for j in self.neighbours:
            self.create_subscription(
                VehicleLocalPosition,
                topic_for_drone(j, 'out/vehicle_local_position'),
                self._make_pos_callback(j),
                qos,
            )

        # Virtual leader
        leader_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            Float32MultiArray, '/leader/state', self.on_leader_state, leader_qos,
        )

        # =====================================================
        # NEW: predicted-trajectory exchange between peers
        # =====================================================
        self.pub_predicted = self.create_publisher(
            Float64MultiArray,
            mpc_topic_for_drone(self.drone_id, 'predicted_trajectory'),
            10,
        )
        for j in self.neighbours:
            self.create_subscription(
                Float64MultiArray,
                mpc_topic_for_drone(j, 'predicted_trajectory'),
                self._make_pred_callback(j),
                10,
            )

        # =====================================================
        # Control timer
        # =====================================================
        self.timer = self.create_timer(1.0 / self.control_hz, self.control_loop)

        self.get_logger().info(
            f'mpc_controller drone {self.drone_id} ready. '
            f'birth={self.birth_positions[self.drone_id]}, '
            f'r_i0={self.my_offset}, '
            f'neighbours={self.neighbours}, '
            f'control_hz={self.control_hz}, mpc N={N} dt={mpc_dt}'
        )

    # =================================================================
    # callbacks
    # =================================================================
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
            # PX4 local frame (origin at spawn) -> world by adding birth offset
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

    def _make_pred_callback(self, drone_idx):
        def cb(msg):
            data = np.asarray(msg.data, dtype=float)
            if data.size % 3 != 0 or data.size == 0:
                return
            traj = data.reshape(-1, 3)
            self.peer_predictions[drone_idx] = traj
        return cb

    # =================================================================
    # control loop
    # =================================================================
    def control_loop(self):
        # 1. heartbeat OffboardControlMode at every tick
        self.publish_offboard_mode()

        # 2. dt sanity
        now_ros = self.get_clock().now()
        dt = (now_ros - self.last_control_time).nanoseconds * 1e-9
        self.last_control_time = now_ros
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / self.control_hz

        # 3. startup phase: stream zero-velocity setpoints (same as v3)
        if self._startup_counter < self.startup_zero_vel_frames:
            self._startup_counter += 1
            self.publish_velocity_setpoint([0.0, 0.0, 0.0], 0.0)
            return

        self_ds = self.drone_states[self.drone_id]
        if not self_ds.received:
            self.publish_velocity_setpoint([0.0, 0.0, 0.0], 0.0)
            return

        # 4. without leader, hold target altitude (same fallback as v3)
        if not self.leader_received:
            z_err = self.target_alt - self_ds.pos[2]
            vz = max(-self.max_climb, min(self.max_climb, 0.8 * z_err))
            self.publish_velocity_setpoint([0.0, 0.0, vz], 0.0)
            return

        # 5. build MPC reference
        x_ref = self._build_reference_trajectory()

        # 6. assemble neighbour predictions (or fall back to current pos)
        nb_traj = self._collect_neighbour_predictions()

        # 7. solve MPC
        x0 = np.concatenate([self_ds.pos, self_ds.vel])
        try:
            u0, x_pred, info = self.mpc.solve(x0, x_ref, nb_traj)
        except Exception as e:
            self.get_logger().warn(
                f'MPC solve crashed: {e}; falling back to zero velocity'
            )
            self.publish_velocity_setpoint([0.0, 0.0, 0.0], 0.0)
            return

        if info['status'] not in (0, 2):
            # status 2 = max-iter reached; result still usable
            self.get_logger().warn_throttle(
                2.0, f'acados status={info["status"]} cost={info["cost"]:.2f}'
            ) if hasattr(self.get_logger(), 'warn_throttle') else \
                self.get_logger().warn(f'acados status={info["status"]}')

        # 8. extract velocity command from one-step-ahead prediction
        v_cmd = x_pred[1, 3:6]
        if not np.all(np.isfinite(v_cmd)):
            self.get_logger().warn('NaN in v_cmd, falling back to zero')
            v_cmd = np.zeros(3)

        # safety clamp (acados constraints should already hold this; double-check)
        v_xy = v_cmd[:2]
        v_xy_norm = float(np.linalg.norm(v_xy))
        if v_xy_norm > self.max_speed:
            v_xy = v_xy * (self.max_speed / v_xy_norm)
        vz = float(np.clip(v_cmd[2], -self.max_climb, self.max_climb))
        v_cmd = np.array([v_xy[0], v_xy[1], vz])

        # 9. yaw — same logic as v3
        if v_xy_norm > 0.5:
            yaw_sp = math.atan2(v_cmd[1], v_cmd[0])
        else:
            yaw_sp = self.leader_yaw

        # 10. publish to PX4 (TrajectorySetpoint, velocity mode — same as v3)
        self.publish_velocity_setpoint(v_cmd.tolist(), yaw_sp)

        # 11. broadcast our predicted trajectory for peers
        self.publish_predicted_trajectory(x_pred[:, 0:3])

    # =================================================================
    # MPC helpers
    # =================================================================
    def _build_reference_trajectory(self):
        """
        Reference at stage k (k=0..N):
            pos_ref[k] = leader_pos + leader_vel * (k * dt) + my_offset
            vel_ref[k] = leader_vel
        Altitude target overrides leader z to keep all drones at target_alt
        (matches v3's separated-z behaviour, but baked into MPC ref).
        """
        N = self.N
        x_ref = np.zeros((N + 1, 6))
        for k in range(N + 1):
            t = k * self.mpc_dt
            pos = self.leader_pos + self.leader_vel * t + self.my_offset
            pos[2] = self.target_alt    # force altitude
            x_ref[k, 0:3] = pos
            x_ref[k, 3:5] = self.leader_vel[:2]   # vz_ref = 0 (vertical hold)
            x_ref[k, 5]   = 0.0
        return x_ref

    def _collect_neighbour_predictions(self):
        """
        Build (M, N+1, 3) tensor of neighbour predicted positions.
        - If peer broadcast a recent prediction, use it (truncate/pad to N+1).
        - Else fall back to peer's current position replicated across stages.
        - If peer hasn't been heard from at all, place it far away (no penalty).
        """
        if not self.neighbours:
            return None
        now = self.get_clock().now().nanoseconds * 1e-9
        N1 = self.N + 1
        out = np.zeros((len(self.neighbours), N1, 3))
        for idx, j in enumerate(self.neighbours):
            traj = self.peer_predictions.get(j)
            if traj is not None and traj.shape[0] >= 1:
                if traj.shape[0] >= N1:
                    out[idx] = traj[:N1]
                else:
                    pad = np.repeat(traj[-1:], N1 - traj.shape[0], axis=0)
                    out[idx] = np.vstack([traj, pad])
                continue

            ds = self.drone_states[j]
            if ds.received and (now - ds.last_stamp) <= self.neighbour_timeout:
                out[idx] = np.tile(ds.pos, (N1, 1))
            else:
                # peer effectively absent — push it far
                out[idx] = np.tile(np.array([1e3, 1e3, 1e3]), (N1, 1))
        return out

    # PX4 publishers
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

    def publish_predicted_trajectory(self, pred_xyz):
        msg = Float64MultiArray()
        rows, cols = pred_xyz.shape
        msg.layout.dim = [
            MultiArrayDimension(label='rows', size=rows, stride=rows * cols),
            MultiArrayDimension(label='cols', size=cols, stride=cols),
        ]
        msg.layout.data_offset = 0
        msg.data = pred_xyz.astype(float).flatten().tolist()
        self.pub_predicted.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MpcControllerNode()
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