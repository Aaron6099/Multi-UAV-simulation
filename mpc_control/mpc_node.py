#!/usr/bin/env python3
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
    VehicleCommand,
    VehicleStatus,
)


# ------------------------------------------------------------------ helpers
def make_px4_qos():
    """订阅 PX4 'out' 话题：PX4 DataWriter 用 TRANSIENT_LOCAL，ROS2 订阅者匹配。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def make_px4_pub_qos():
    """发布到 PX4 'in' 话题：PX4 DataReader 用 VOLATILE，ROS2 发布者必须匹配。
    用 TRANSIENT_LOCAL 会导致多机时消息丢失、OFFBOARD 丢失、ARM 失败。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
        durability=DurabilityPolicy.VOLATILE,
    )


def quaternion_to_yaw(q):
    w, x, y, z = q[0], q[1], q[2], q[3]
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def safe_finite(x, default=0.0):
    return float(x) if math.isfinite(x) else float(default)


def topic_for_drone(drone_id, suffix):
    if drone_id == 0:
        return f'/fmu/{suffix}'
    return f'/px4_{drone_id}/fmu/{suffix}'


def mpc_topic_for_drone(drone_id, suffix='predicted_trajectory'):
    if drone_id == 0:
        return f'/mpc/{suffix}'
    return f'/px4_{drone_id}/mpc/{suffix}'


# ------------------------------------------------------------------ MPC core
class DoubleIntegratorMPC:
    def __init__(self, N=20, dt=0.05, max_speed=5.0, max_climb=1.5,
                 max_accel=5.0, max_neighbours=4, d_safe=1.2,
                 w_collision=200.0, w_formation=0.5,
                 q_pos=4.0, q_vel=1.0, r_acc=0.1,
                 q_pos_terminal_scale=2.0,
                 build_dir='/tmp/acados_di_mpc', instance_id=0):
        self.N = N; self.dt = dt
        self.max_speed = max_speed; self.max_climb = max_climb
        self.max_accel = max_accel
        self.max_neighbours = max(1, int(max_neighbours))
        self.d_safe = d_safe
        self.w_collision = w_collision; self.w_formation = w_formation
        self.q_pos = q_pos; self.q_vel = q_vel; self.r_acc = r_acc
        self.q_pos_terminal_scale = q_pos_terminal_scale

        _m = max(1, int(max_neighbours))
        self._build_dir = f'{build_dir}_v{instance_id}_m{_m}'
        os.makedirs(self._build_dir, exist_ok=True)
        self._instance_id = instance_id

        self.solver = None
        self._setup_ocp()

    def _setup_ocp(self):
        import casadi as ca
        from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

        nx, nu = 6, 3
        x = ca.SX.sym('x', nx)
        u = ca.SX.sym('u', nu)

        M = self.max_neighbours
        p_pos    = ca.SX.sym('p_pos',  3 * M)
        p_active = ca.SX.sym('p_act',      M)
        p_dstar  = ca.SX.sym('p_dstar',    M)
        p_full = ca.vertcat(p_pos, p_active, p_dstar)

        f_expl = ca.vertcat(x[3], x[4], x[5], u[0], u[1], u[2])

        model = AcadosModel()
        model.name = f'di_mpc_i{self._instance_id}_m{M}'
        model.x = x; model.u = u; model.p = p_full
        model.f_expl_expr = f_expl
        xdot = ca.SX.sym('xdot', nx)
        model.xdot = xdot
        model.f_impl_expr = xdot - f_expl

        ocp = AcadosOcp()
        ocp.model = model
        ocp.dims.N = self.N

        coll_residuals = []
        form_residuals = []
        for i in range(M):
            ni = p_pos[3*i : 3*(i+1)]
            diff_xy = x[0:2] - ni[0:2]
            d2 = ca.sumsqr(diff_xy) + 1e-6
            d_i = ca.sqrt(d2)
            active = p_active[i]
            d_star = p_dstar[i]
            coll_residuals.append(
                ca.sqrt(self.w_collision) * active * ca.fmax(0.0, self.d_safe - d_i)
            )
            form_residuals.append(
                ca.sqrt(self.w_formation) * active * (d_i - d_star)
            )

        y_track = ca.vertcat(x[0:3], x[3:6], u)
        y_coll  = ca.vertcat(*coll_residuals) if M > 0 else ca.SX.zeros(0, 1)
        y_form  = ca.vertcat(*form_residuals) if M > 0 else ca.SX.zeros(0, 1)
        y_expr  = ca.vertcat(y_track, y_coll, y_form)
        y_expr_e = ca.vertcat(x[0:3], x[3:6])

        ny = 9 + 2 * M

        ocp.cost.cost_type = 'NONLINEAR_LS'
        ocp.cost.cost_type_e = 'NONLINEAR_LS'
        ocp.model.cost_y_expr = y_expr
        ocp.model.cost_y_expr_e = y_expr_e

        w_diag = (
            [self.q_pos]*3 + [self.q_vel]*3 + [self.r_acc]*3 +
            [1.0]*M + [1.0]*M
        )
        ocp.cost.W = np.diag(w_diag)
        ocp.cost.W_e = np.diag(
            [self.q_pos * self.q_pos_terminal_scale]*3 + [self.q_vel]*3
        )
        ocp.cost.yref   = np.zeros(ny)
        ocp.cost.yref_e = np.zeros(6)

        ocp.constraints.lbu = np.array([-self.max_accel]*3)
        ocp.constraints.ubu = np.array([+self.max_accel]*3)
        ocp.constraints.idxbu = np.arange(nu)
        ocp.constraints.lbx = np.array([-self.max_speed, -self.max_speed, -self.max_climb])
        ocp.constraints.ubx = np.array([+self.max_speed, +self.max_speed, +self.max_climb])
        ocp.constraints.idxbx = np.array([3, 4, 5])
        ocp.constraints.x0 = np.zeros(nx)
        ocp.parameter_values = np.zeros(p_full.shape[0])

        ocp.solver_options.tf = self.N * self.dt
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.sim_method_num_stages = 4
        ocp.solver_options.sim_method_num_steps = 1
        ocp.solver_options.print_level = 0
        ocp.solver_options.qp_solver_iter_max = 100
        ocp.solver_options.qp_solver_warm_start = 1
        ocp.solver_options.levenberg_marquardt = 1e-4
        ocp.solver_options.nlp_solver_max_iter = 30

        ocp.code_export_directory = os.path.join(self._build_dir, 'c_generated_code')
        json_file = os.path.join(self._build_dir, 'acados_ocp.json')
        self.solver = AcadosOcpSolver(ocp, json_file=json_file)
        self._nx, self._nu = nx, nu

    def solve(self, x0, x_ref, neighbour_traj=None, desired_distances=None):
        N = self.N; M = self.max_neighbours
        self.solver.set(0, 'lbx', x0)
        self.solver.set(0, 'ubx', x0)
        for k in range(N):
            yref_k = np.concatenate([
                x_ref[k, 0:3], x_ref[k, 3:6], np.zeros(3),
                np.zeros(M), np.zeros(M),
            ])
            self.solver.set(k, 'yref', yref_k)
            self.solver.set(k, 'p', self._pack_params(k, neighbour_traj, desired_distances))
        self.solver.set(N, 'yref', x_ref[N, 0:6])
        self.solver.set(N, 'p', self._pack_params(N, neighbour_traj, desired_distances))
        status = self.solver.solve()
        u0 = self.solver.get(0, 'u')
        x_pred = np.zeros((N + 1, self._nx))
        for k in range(N + 1):
            x_pred[k, :] = self.solver.get(k, 'x')
        return u0, x_pred, {
            'status': int(status),
            'cost': float(self.solver.get_cost()),
            'solve_time_s': float(self.solver.get_stats('time_tot')),
        }

    def _pack_params(self, k, neighbour_traj, desired_distances):
        M = self.max_neighbours
        p = np.zeros(3 * M + M + M)
        if neighbour_traj is None:
            return p
        nt = np.asarray(neighbour_traj)
        if nt.ndim != 3 or nt.shape[2] != 3:
            return p
        M_real = min(nt.shape[0], M)
        K = max(0, nt.shape[1] - 1)
        kc = min(k, K)
        dd = (np.asarray(desired_distances) if desired_distances is not None
              else np.zeros(M_real))
        for i in range(M_real):
            p[3*i : 3*(i+1)] = nt[i, kc, :]
            p[3*M + i]       = 1.0
            if i < len(dd):
                p[4*M + i]   = float(dd[i])
        return p


# -- state
class DroneState:
    def __init__(self):
        self.received = False
        self.last_stamp = 0.0
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.yaw = 0.0


# -- node
class MpcControllerNode(Node):
    def __init__(self):
        super().__init__('mpc_controller_node')

        self.declare_parameter('drone_id', 0)
        self.declare_parameter('num_drones', 9)

        # 默认出生位置（NED: x=北, y=东），与 swarm_launch.py 的 BIRTH_9 一致
        default_births = [
            0.0,  0.0, 0.0,   # 0 中心
            0.0,  3.0, 0.0,   # 1 东
            0.0, -3.0, 0.0,   # 2 西
            3.0,  0.0, 0.0,   # 3 北
           -3.0,  0.0, 0.0,   # 4 南
            3.0,  3.0, 0.0,   # 5 东北
           -3.0,  3.0, 0.0,   # 6 东南
            3.0, -3.0, 0.0,   # 7 西北
           -3.0, -3.0, 0.0,   # 8 西南
        ]
        self.declare_parameter('birth_positions_flat', default_births)
        self.declare_parameter('formation_offsets_flat', default_births)
        self.declare_parameter('neighbours', [0])

        self.declare_parameter('target_alt', -5.0)
        self.declare_parameter('max_speed', 5.0)
        self.declare_parameter('max_climb', 1.5)
        self.declare_parameter('max_accel', 5.0)
        self.declare_parameter('control_hz', 50.0)
        self.declare_parameter('neighbour_timeout', 1.0)
        self.declare_parameter('startup_zero_vel_frames', 30)

        self.declare_parameter('mpc_horizon', 20)
        self.declare_parameter('mpc_dt', 0.05)
        self.declare_parameter('q_pos', 4.0)
        self.declare_parameter('q_vel', 1.0)
        self.declare_parameter('r_acc', 0.1)
        self.declare_parameter('q_pos_terminal_scale', 2.0)
        self.declare_parameter('d_safe', 1.2)
        self.declare_parameter('w_collision', 200.0)
        self.declare_parameter('w_formation', 0.5)
        self.declare_parameter('acados_build_dir', '/tmp/acados_di_mpc')

        self.drone_id   = int(self.get_parameter('drone_id').value)
        self.num_drones = int(self.get_parameter('num_drones').value)

        births = list(self.get_parameter('birth_positions_flat').value)
        if len(births) != 3 * self.num_drones:
            raise RuntimeError(
                f'birth_positions_flat must have {3*self.num_drones} elements')
        self.birth_positions = np.array(births, dtype=float).reshape(self.num_drones, 3)
        # DYNAMIC version — mutated on each EKF reset
        self.world_birth = self.birth_positions.copy()

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

        self.desired_distances = np.array([
            float(np.linalg.norm(
                self.my_offset[:2] - self.formation_offsets[j][:2]
            ))
            for j in self.neighbours
        ])

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
        w_form   = float(self.get_parameter('w_formation').value)
        build_dir = str(self.get_parameter('acados_build_dir').value)

        # State
        self.drone_states = [DroneState() for _ in range(self.num_drones)]
        self.leader_received = False
        self.leader_pos = np.zeros(3)
        self.leader_vel = np.zeros(3)
        self.leader_yaw = 0.0
        self.attitude_yaw = 0.0
        self.attitude_received = False
        self.last_control_time = self.get_clock().now()
        self._startup_counter = 0
        self.peer_predictions = {}
        self.peer_prediction_stamps = {}   # drone_id -> float (seconds)
        self._dbg_counter = 0

        self.last_valid_yaw = 0.0

        # 健康诊断计数器
        self._fallback_count = 0           # 累计 hover 降级次数
        self._hover_active = False         # 当前帧是否在 hover 降级
        self._last_mpc_status = 0
        self._last_solve_ms = 0.0
        self._last_pos_err = 0.0

        # EKF reset trackers (one per known drone)
        self._prev_xy_reset = [0] * self.num_drones
        self._prev_z_reset  = [0] * self.num_drones
        self._pos_calibrated = [False] * self.num_drones

        # MPC
        self.N = N
        self.mpc_dt = mpc_dt
        self.get_logger().info(
            f'building acados OCP for drone {self.drone_id} '
            f'(N={N}, dt={mpc_dt}, neighbours={self.neighbours}, '
            f'd_star={self.desired_distances.tolist()})...'
        )
        self.mpc = DoubleIntegratorMPC(
            N=N, dt=mpc_dt,
            max_speed=self.max_speed,
            max_climb=self.max_climb,
            max_accel=self.max_accel,
            max_neighbours=max(1, len(self.neighbours)),
            d_safe=d_safe,
            w_collision=w_coll,
            w_formation=w_form,
            q_pos=q_pos, q_vel=q_vel, r_acc=r_acc,
            q_pos_terminal_scale=q_term_s,
            build_dir=build_dir,
            instance_id=self.drone_id,
        )
        self.get_logger().info('acados OCP ready.')

        # ROS 2 IO
        qos     = make_px4_qos()      # 订阅 PX4 "out" 话题（TRANSIENT_LOCAL）
        pub_qos = make_px4_pub_qos()  # 发布到 PX4 "in" 话题（VOLATILE，必须匹配 PX4 DataReader）
        self.pub_offboard_mode = self.create_publisher(
            OffboardControlMode,
            topic_for_drone(self.drone_id, 'in/offboard_control_mode'), pub_qos,
        )
        self.pub_setpoint = self.create_publisher(
            TrajectorySetpoint,
            topic_for_drone(self.drone_id, 'in/trajectory_setpoint'), pub_qos,
        )
        self.pub_vehicle_cmd = self.create_publisher(
            VehicleCommand,
            topic_for_drone(self.drone_id, 'in/vehicle_command'), pub_qos,
        )
        self._arming_state = 0   # 0=unknown,1=disarmed,2=armed
        self._nav_state   = 0
        self._arm_offboard_confirmed = False
        self._cmd_retry_counter = 0   # frames since startup finished
        self.create_subscription(
            VehicleStatus,
            topic_for_drone(self.drone_id, 'out/vehicle_status'),
            self._on_vehicle_status, qos,
        )
        self.create_subscription(
            VehicleAttitude,
            topic_for_drone(self.drone_id, 'out/vehicle_attitude'),
            self.on_self_attitude, qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            topic_for_drone(self.drone_id, 'out/vehicle_local_position'),
            self._make_pos_callback(self.drone_id), qos,
        )
        for j in self.neighbours:
            self.create_subscription(
                VehicleLocalPosition,
                topic_for_drone(j, 'out/vehicle_local_position'),
                self._make_pos_callback(j), qos,
            )
        leader_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            Float64MultiArray, '/leader/state', self.on_leader_state, leader_qos,
        )
        self.pub_predicted = self.create_publisher(
            Float64MultiArray,
            mpc_topic_for_drone(self.drone_id, 'predicted_trajectory'), 10,
        )
        # 健康诊断话题（供 diag_monitor.py 订阅）
        # 格式: [drone_id, mpc_status, solve_ms, fallback_count, hover_active, pos_err_m]
        self.pub_health = self.create_publisher(
            Float32MultiArray,
            mpc_topic_for_drone(self.drone_id, 'health'), 10,
        )
        for j in self.neighbours:
            self.create_subscription(
                Float64MultiArray,
                mpc_topic_for_drone(j, 'predicted_trajectory'),
                self._make_pred_callback(j), 10,
            )
        self.timer = self.create_timer(1.0 / self.control_hz, self.control_loop)

        self.get_logger().info(
            f'mpc_controller drone {self.drone_id} ready. '
            f'birth={self.birth_positions[self.drone_id]}, '
            f'r_i0={self.my_offset}, '
            f'neighbours={self.neighbours}'
        )

    # =================================================================
    # callbacks
    # =================================================================
    def _make_pos_callback(self, drone_idx):
        def cb(msg):
            now = self.get_clock().now().nanoseconds * 1e-9
            ds = self.drone_states[drone_idx]

            # --- detect EKF reset; update dynamic birth offset ---
            # Skip until first position calibrated world_birth — stale resets
            # from before MPC startup would corrupt the offset.
            if self._pos_calibrated[drone_idx]:
                if msg.xy_reset_counter > self._prev_xy_reset[drone_idx]:
                    self.world_birth[drone_idx, 0] -= float(msg.delta_xy[0])
                    self.world_birth[drone_idx, 1] -= float(msg.delta_xy[1])
                    self.get_logger().warn(
                        f'[veh {drone_idx}] xy reset #{msg.xy_reset_counter}, '
                        f'delta=({msg.delta_xy[0]:+.2f}, {msg.delta_xy[1]:+.2f}); '
                        f'world_birth -> ({self.world_birth[drone_idx,0]:+.2f}, '
                        f'{self.world_birth[drone_idx,1]:+.2f})'
                    )
                    self._prev_xy_reset[drone_idx] = msg.xy_reset_counter
                if msg.z_reset_counter > self._prev_z_reset[drone_idx]:
                    self.world_birth[drone_idx, 2] -= float(msg.delta_z)
                    self.get_logger().warn(
                        f'[veh {drone_idx}] z reset #{msg.z_reset_counter}, '
                        f'delta={msg.delta_z:+.2f}; '
                        f'world_birth z -> {self.world_birth[drone_idx,2]:+.2f}'
                    )
                    self._prev_z_reset[drone_idx] = msg.z_reset_counter

            if not ds.received:
                # Calibrate world_birth: set so that first world_pos = birth_pos.
                first_local = np.array([msg.x, msg.y, msg.z])
                self.world_birth[drone_idx] = (
                    self.birth_positions[drone_idx] - first_local
                )
                self._prev_xy_reset[drone_idx] = msg.xy_reset_counter
                self._prev_z_reset[drone_idx] = msg.z_reset_counter
                self._pos_calibrated[drone_idx] = True
                self.get_logger().info(
                    f'first position from drone {drone_idx}: '
                    f'local=({msg.x:.2f}, {msg.y:.2f}, {msg.z:.2f}) '
                    f'world_birth=({self.world_birth[drone_idx,0]:.2f}, '
                    f'{self.world_birth[drone_idx,1]:.2f}, '
                    f'{self.world_birth[drone_idx,2]:.2f})'
                )
            ds.received = True
            ds.last_stamp = now
            # Use DYNAMIC world_birth (compensates for EKF resets)
            ds.pos = np.array([msg.x, msg.y, msg.z]) + self.world_birth[drone_idx]
            ds.vel = np.array([msg.vx, msg.vy, msg.vz])
            ds.yaw = float(msg.heading) if math.isfinite(msg.heading) else 0.0
        return cb

    def _on_vehicle_status(self, msg):
        self._arming_state = msg.arming_state
        self._nav_state    = msg.nav_state

    def _send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command          = command
        msg.param1           = float(param1)
        msg.param2           = float(param2)
        msg.target_system    = self.drone_id + 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_vehicle_cmd.publish(msg)

    def _arm_and_engage_offboard(self):
        """Retry ARM + OFFBOARD until confirmed. Call every ~100 frames (2 s at 50 Hz)."""
        if self._arm_offboard_confirmed:
            return
        if self._nav_state != 14:
            self._send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        if self._arming_state != 2:
            self._send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        if self._nav_state == 14 and self._arming_state == 2:
            self._arm_offboard_confirmed = True
            self.get_logger().info(
                f'drone {self.drone_id}: OFFBOARD + ARMED confirmed')

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
            self.peer_prediction_stamps[drone_idx] = self.get_clock().now().nanoseconds * 1e-9
        return cb

    # =================================================================
    # control loop
    # =================================================================
    def control_loop(self):
        self.publish_offboard_mode()
        now_ros = self.get_clock().now()
        dt = (now_ros - self.last_control_time).nanoseconds * 1e-9
        self.last_control_time = now_ros
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / self.control_hz

        # 使用当前姿态 yaw，避免起飞时强制转向正北
        yaw_hold = self.attitude_yaw if self.attitude_received else 0.0

        if self._startup_counter < self.startup_zero_vel_frames:
            self._startup_counter += 1
            self._hover_active = True
            self.publish_position_setpoint(self._hover_setpoint_world(), np.zeros(3), yaw_hold)
            return

        # After startup: retry ARM+OFFBOARD every 50 frames (1 s) until confirmed
        if not self._arm_offboard_confirmed:
            self._cmd_retry_counter += 1
            if self._cmd_retry_counter % 50 == 1:
                self._arm_and_engage_offboard()
                if not self._arm_offboard_confirmed:
                    self.get_logger().info(
                        f'drone {self.drone_id}: retry ARM+OFFBOARD '
                        f'(nav={self._nav_state}, arm={self._arming_state}) '
                        f'frame={self._cmd_retry_counter}')
            self._hover_active = True
            self.publish_position_setpoint(self._hover_setpoint_world(), np.zeros(3), yaw_hold)
            return

        self_ds = self.drone_states[self.drone_id]
        if not self_ds.received:
            self._hover_active = True
            self.publish_position_setpoint(self._hover_setpoint_world(), np.zeros(3), yaw_hold)
            return

        if not self.leader_received:
            self._hover_active = True
            self.publish_position_setpoint(self._hover_setpoint_world(), np.zeros(3), yaw_hold)
            return

        x_ref = self._build_reference_trajectory()
        nb_traj = self._collect_neighbour_predictions()
        x0 = np.concatenate([self_ds.pos, self_ds.vel])

        self._hover_active = False   # 假设本帧正常，后续如有降级会覆盖为 True

        try:
            u0, x_pred, info = self.mpc.solve(
                x0, x_ref, nb_traj,
                desired_distances=self.desired_distances,
            )
        except Exception as e:
            self.get_logger().warn(f'MPC solve crashed: {e}; holding position')
            self._fallback_count += 1
            self._hover_active = True
            self._publish_health()
            self.publish_position_setpoint(self._hover_setpoint_world(), np.zeros(3), 0.0)
            return

        self._last_mpc_status = int(info['status'])
        self._last_solve_ms   = float(info['solve_time_s']) * 1000.0

        self._dbg_counter += 1
        if self._dbg_counter <= 5:
            self.get_logger().warn(
                f'DEBUG d{self.drone_id}: '
                f'x0=({x0[0]:.2f},{x0[1]:.2f},{x0[2]:.2f}) '
                f'ref0=({x_ref[0,0]:.2f},{x_ref[0,1]:.2f},{x_ref[0,2]:.2f}) '
                f'pred1=({x_pred[1,0]:.2f},{x_pred[1,1]:.2f},{x_pred[1,2]:.2f}) '
                f'leader=({self.leader_pos[0]:.2f},{self.leader_pos[1]:.2f},{self.leader_pos[2]:.2f})'
            )
        if self._dbg_counter % int(self.control_hz) == 0:
            self.get_logger().info(
                f'[d{self.drone_id}] solve: status={info["status"]} '
                f'time={self._last_solve_ms:.2f}ms '
                f'cost={info["cost"]:.1f} '
                f'pos=({self_ds.pos[0]:.2f},{self_ds.pos[1]:.2f},{self_ds.pos[2]:.2f}) '
                f'fallbacks={self._fallback_count}'
            )

        yaw_sp = self.leader_yaw if math.isfinite(self.leader_yaw) else 0.0

        # 0=成功, 2=达到最大迭代但解仍可用; 1=发散, 3=最小步长, 4=QP失败 → 位置保持
        if info['status'] not in (0, 2):
            self.get_logger().warn(
                f'[d{self.drone_id}] acados status={info["status"]} — holding position '
                f'(total fallbacks={self._fallback_count + 1})'
            )
            self._fallback_count += 1
            self._hover_active = True
            self._publish_health()
            self.publish_position_setpoint(self._hover_setpoint_world(), np.zeros(3), yaw_sp)
            return

        # ── Velocity control mode ──
        # MPC outputs acceleration u0; integrate to get velocity command.
        # Position P-controller for z-axis (altitude hold).
        u0 = x_pred[0, 0:3]  # actually this is x_pred[0] state, need u0 from solver
        # Use predicted velocity at k=1 as base, plus position error correction
        pred_vel = x_pred[1, 3:6].copy()

        # Position error → velocity correction (P-controller)
        ref_pos = x_ref[0, 0:3]
        pos_err_vec = ref_pos - self_ds.pos
        Kp_pos = 1.0  # position gain (m/s per m of error)
        vel_correction = np.clip(pos_err_vec * Kp_pos, -self.max_speed, self.max_speed)

        # Blend: MPC velocity + position correction
        # For horizontal: use MPC velocity (dynamic tracking)
        # For vertical: use position correction (altitude hold)
        vel_sp = np.zeros(3)
        vel_sp[0] = pred_vel[0] + vel_correction[0] * 0.5  # MPC dominant
        vel_sp[1] = pred_vel[1] + vel_correction[1] * 0.5  # MPC dominant
        vel_sp[2] = vel_correction[2]  # pure P-controller for z

        # Clip to limits
        vel_xy_norm = float(np.linalg.norm(vel_sp[:2]))
        if vel_xy_norm > self.max_speed:
            vel_sp[:2] *= self.max_speed / vel_xy_norm
        vel_sp[2] = np.clip(vel_sp[2], -self.max_climb, self.max_climb)

        # Safety: NaN check
        if not np.all(np.isfinite(vel_sp)):
            vel_sp = np.zeros(3)

        # Position error for logging
        pos_err = float(np.linalg.norm(pos_err_vec[:2]))
        self._last_pos_err = pos_err

        self._publish_health()
        self.publish_velocity_setpoint(vel_sp, yaw_sp)
        self.publish_predicted_trajectory(x_pred[:, 0:3])

    def _build_reference_trajectory(self):
        N = self.N
        x_ref = np.zeros((N + 1, 6))
        leader_pos_safe = (self.leader_pos.copy()
                           if np.all(np.isfinite(self.leader_pos)) else np.zeros(3))
        leader_vel_safe = (self.leader_vel.copy()
                           if np.all(np.isfinite(self.leader_vel)) else np.zeros(3))
        for k in range(N + 1):
            t = k * self.mpc_dt
            pos = leader_pos_safe + leader_vel_safe * t + self.my_offset
            pos[2] = self.target_alt
            x_ref[k, 0:3] = pos
            x_ref[k, 3:5] = leader_vel_safe[:2]
            x_ref[k, 5]   = 0.0
        return x_ref

    def _collect_neighbour_predictions(self):
        if not self.neighbours:
            return None
        now = self.get_clock().now().nanoseconds * 1e-9
        N1 = self.N + 1
        out = np.zeros((len(self.neighbours), N1, 3))
        for idx, j in enumerate(self.neighbours):
            traj = self.peer_predictions.get(j)
            stamp = self.peer_prediction_stamps.get(j, 0.0)
            pred_fresh = (traj is not None and traj.shape[0] >= 1
                          and (now - stamp) <= self.neighbour_timeout)
            if pred_fresh:
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
                out[idx] = np.tile(self.drone_states[self.drone_id].pos + self.formation_offsets[j] - self.formation_offsets[self.drone_id], (N1, 1))
        return out

    def _publish_health(self):
        """发布 MPC 健康诊断数据，供 diag_monitor.py 实时监控。
        格式: [drone_id, mpc_status, solve_ms, fallback_count, hover_active, pos_err_m]"""
        msg = Float32MultiArray()
        msg.data = [
            float(self.drone_id),
            float(self._last_mpc_status),
            float(self._last_solve_ms),
            float(self._fallback_count),
            float(1 if self._hover_active else 0),
            float(self._last_pos_err),
        ]
        self.pub_health.publish(msg)

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True   # 速度控制模式：与 publish_velocity_setpoint 匹配
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_offboard_mode.publish(msg)

    def _hover_setpoint_world(self):
        """当前 XY 保持（有位置时）或出生点 XY，高度收敛到目标高度，均为世界系 NED。"""
        ds = self.drone_states[self.drone_id]
        if ds.received:
            return np.array([ds.pos[0], ds.pos[1], self.target_alt])
        birth = self.world_birth[self.drone_id]
        return np.array([birth[0], birth[1], self.target_alt])

    def publish_position_setpoint(self, pos_world_ned, vel_ff_world_ned, yaw):
        """位置闭环 + 速度前馈（PX4 官方推荐：position≠NaN时，velocity作前馈项）。
        pos_world_ned / vel_ff_world_ned 均为世界系 NED；内部自动转换到 PX4 本地系。"""
        my_birth = self.world_birth[self.drone_id]
        # 世界系 → PX4本地系（平移不影响速度方向，直接传速度即可）
        pos_local = pos_world_ned - my_birth
        nan3 = [float('nan')] * 3
        msg = TrajectorySetpoint()
        msg.position = [safe_finite(pos_local[0]), safe_finite(pos_local[1]), safe_finite(pos_local[2])]
        msg.velocity = [safe_finite(v, float('nan')) for v in vel_ff_world_ned]
        msg.acceleration = nan3
        msg.yaw = safe_finite(yaw, 0.0)
        msg.yawspeed = float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_setpoint.publish(msg)

    def publish_velocity_setpoint(self, vel_world_ned, yaw):
        """保留兼容性；正常运行路径已不调用此方法。"""
        vx = safe_finite(vel_world_ned[0], 0.0)
        vy = safe_finite(vel_world_ned[1], 0.0)
        vz = safe_finite(vel_world_ned[2], 0.0)
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [vx, vy, vz]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        msg.yaw = safe_finite(yaw, 0.0)
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