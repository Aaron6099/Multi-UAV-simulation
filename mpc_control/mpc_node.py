#!/usr/bin/env python3
import math
import os
import random
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
)

from std_msgs.msg import Float32MultiArray, Float64MultiArray, MultiArrayDimension, Float64
from px4_msgs.msg import (
    VehicleLocalPosition,
    VehicleAttitude,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
)

from mpc_control.safety_filter import SafetyFilter


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
        self.xy_valid = False   # EKF 水平估计健康（safety_filter 估计门用）
        self.z_valid = False    # EKF 垂直估计健康


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
        # 校准 world_birth 前,要求 EKF 连续 valid 的帧数(等 GPS fix 收敛)
        self.declare_parameter('calib_settle_frames', 25)
        # 校准窗口：只在 local 连续 calib_stable_window 帧"极差 < tol"时才锁定，
        # 不看绝对值(对恒定偏置也成立)，x/y/z 统一生效
        self.declare_parameter('calib_stable_window', 40)     # 稳定性判定窗口(帧)
        self.declare_parameter('calib_stable_tol',    0.05)   # m，窗口三轴极差上限
        self.declare_parameter('calib_timeout_frames', 600)   # 兜底，防永不收敛死锁
        # EKF 收敛后静止于自身 local 原点，|local_xy| 必接近 0；仍几十米=GPS fix 前暂态，拒绝锁定
        self.declare_parameter('calib_max_origin_offset', 2.0)  # m，校准锁定时 |local_xy| 上限
        # Tier2: ref_alt 连续温漂(不走 z_reset)→ 持续把 world_birth_z 拉回全员基准
        self.declare_parameter('alt_sync_enable', True)       # 初始用 ref_alt 差校准 world_birth_z(真机各机home海拔不同需要)；SITL 三机同地面应关(ref_alt 差是EKF噪声,补偿会让各机飞到不同物理高度)
        self.declare_parameter('alt_resync_enable', True)
        self.declare_parameter('alt_resync_rate', 0.05)       # m/s，world_birth_z 逼近限速(防 MPC z 跳)
        self.declare_parameter('alt_ref_filter_alpha', 0.05)  # 当前 ref_alt 的 EMA 系数
        self.declare_parameter('alt_resync_max', 3.0)         # m，单机 z 偏移安全上限
        # 悬停期一次性高度 trim：各机 home ref_alt 不同 → 同控 -5 真高散；用 ref_alt 与
        # drone0 datum 之差偏调 target_alt(而非 world_birth_z，避开 alt_resync 撞机根因)，
        # 使各机收敛到同一绝对海拔=同真高；起飞前(leader 起动)冻结，圆周中不再变。
        self.declare_parameter('alt_trim_enable', False)
        self.declare_parameter('alt_trim_max', 2.0)           # m，trim 安全上限

        self.declare_parameter('mpc_horizon', 20)
        self.declare_parameter('mpc_dt', 0.05)
        self.declare_parameter('q_pos', 4.0)
        self.declare_parameter('q_vel', 1.0)
        self.declare_parameter('r_acc', 0.1)
        self.declare_parameter('q_pos_terminal_scale', 2.0)
        self.declare_parameter('d_safe', 1.2)
        self.declare_parameter('w_collision', 200.0)
        self.declare_parameter('w_formation', 0.5)
        # 候选①：XY 位置反馈（前馈 + 有界 P）。kp=0 → 纯前馈(现状/1c4bc5a 行为)；
        # >0 时叠加饱和限幅的位置纠偏，抓住运动时碰撞约束推离槽位的漂移、防发散。
        # cap 限幅避免无阻尼比例环的圆周震荡。非 acados 烤死参数，改后无需清缓存。
        self.declare_parameter('vel_xy_kp', 0.0)
        self.declare_parameter('vel_xy_cap', 0.8)
        self.declare_parameter('acados_build_dir', '/tmp/acados_di_mpc')
        # P2 故障注入：邻居预测轨迹的通信劣化（S16；0=关闭）。
        # delay 注入后走既有时间对齐路径（latency 平移），dropout 直接丢弃消息——
        # 超过 neighbour_timeout 自动落入"常值外推/队形推断"降级，与真失联同路径。
        self.declare_parameter('comms_delay_ms', 0.0)
        self.declare_parameter('comms_dropout', 0.0)
        # 真机安全门控：false = 节点不发 ARM/OFFBOARD 指令，只持续发 setpoint 流并
        # 等待飞手用 RC 解锁+切 OFFBOARD（PX4 要求切换前 setpoint 流已 >2Hz，满足）。
        # SITL 无 RC，保持默认 true 自动解锁。
        self.declare_parameter('auto_arm_enable', True)

        # companion 安全滤波层（缺省保守；真机经 launch 覆盖，d_emergency 应 < d_safe）
        self.declare_parameter('safety_filter_enable', True)
        self.declare_parameter('safety_max_track_dist', 5.0)   # m，偏离参考点上限=飞散阈值
        self.declare_parameter('safety_max_alt', 8.0)          # m，最大离地绝对高度
        self.declare_parameter('safety_min_alt', 0.3)          # m，最小离地绝对高度
        self.declare_parameter('safety_d_emergency', 1.2)      # m，硬碰撞地板（< d_safe）
        self.declare_parameter('safety_self_timeout', 0.3)     # s，自机 EKF 话题新鲜度门限（独立于邻居超时）

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
        self.vel_xy_kp  = float(self.get_parameter('vel_xy_kp').value)
        self.vel_xy_cap = float(self.get_parameter('vel_xy_cap').value)
        self.max_accel  = float(self.get_parameter('max_accel').value)
        self.control_hz = float(self.get_parameter('control_hz').value)
        self.neighbour_timeout = float(self.get_parameter('neighbour_timeout').value)
        self.startup_zero_vel_frames = int(self.get_parameter('startup_zero_vel_frames').value)
        self.calib_settle_frames = max(1, int(self.get_parameter('calib_settle_frames').value))
        self.calib_stable_window = max(2, int(self.get_parameter('calib_stable_window').value))
        self.calib_stable_tol    = float(self.get_parameter('calib_stable_tol').value)
        self.calib_timeout_frames = max(self.calib_settle_frames,
                                        int(self.get_parameter('calib_timeout_frames').value))
        self.calib_max_origin_offset = float(self.get_parameter('calib_max_origin_offset').value)

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
        self.comms_delay_s = max(0.0, float(self.get_parameter('comms_delay_ms').value)) * 1e-3
        self.comms_dropout = min(1.0, max(0.0, float(self.get_parameter('comms_dropout').value)))
        self.auto_arm_enable = bool(self.get_parameter('auto_arm_enable').value)
        if not self.auto_arm_enable:
            self.get_logger().warn(
                f'drone {self.drone_id}: auto_arm DISABLED — 等待飞手 RC 解锁并切 OFFBOARD')
        if self.comms_delay_s > 0.0 or self.comms_dropout > 0.0:
            self.get_logger().warn(
                f'[P2 FAULT INJECTION] comms_delay={self.comms_delay_s*1e3:.0f}ms '
                f'dropout={self.comms_dropout:.0%} — 仅用于降级测试，真机部署必须为 0')

        # State
        self.drone_states = [DroneState() for _ in range(self.num_drones)]
        self.leader_received = False
        self.leader_pos = np.zeros(3)
        self.leader_vel = np.zeros(3)
        self.leader_acc = np.zeros(3)   # 向心加速度（圆周运动）
        self.leader_yaw = 0.0
        self.attitude_yaw = 0.0
        self.attitude_received = False
        self.last_control_time = self.get_clock().now()
        self._startup_counter = 0
        self.peer_predictions = {}
        self.peer_prediction_stamps = {}   # drone_id -> float (seconds)
        self._pred_delay_buf = {}          # P2 注入延迟: drone_id -> [(arrival_t, traj), ...]
        self._dbg_counter = 0

        self.last_valid_yaw = 0.0

        # 健康诊断计数器
        self._fallback_count = 0           # 累计 hover 降级次数
        self._hover_active = False         # 当前帧是否在 hover 降级
        self._last_mpc_status = 0
        self._last_solve_ms = 0.0
        self._last_pos_err = 0.0
        self._last_z_err   = 0.0
        self._ocp_ready = False  # acados 编译完成前不尝试 ARM

        # EKF reset trackers (one per known drone)
        self._prev_xy_reset = [0] * self.num_drones
        self._prev_z_reset  = [0] * self.num_drones
        self._pos_calibrated = [False] * self.num_drones
        self._ref_alt = [None] * self.num_drones   # 各机 EKF 参考海拔 (ref_alt from local_pos)
        # Tier2: 当前(滤波)ref_alt，每帧更新；与上面"标定时冻结的 _ref_alt"区分
        self._ref_alt_now   = [None] * self.num_drones
        self._datum_ref_alt = None   # drone0 广播的当前基准 ref_alt
        self._alt_sync_enable = bool(self.get_parameter('alt_sync_enable').value)
        self._alt_resync_enable = bool(self.get_parameter('alt_resync_enable').value)
        self._alt_resync_rate   = float(self.get_parameter('alt_resync_rate').value)
        self._alt_ref_alpha     = float(self.get_parameter('alt_ref_filter_alpha').value)
        self._alt_resync_max    = float(self.get_parameter('alt_resync_max').value)
        # 悬停期高度 trim 状态
        self._alt_trim_enable = bool(self.get_parameter('alt_trim_enable').value)
        self._alt_trim_max    = float(self.get_parameter('alt_trim_max').value)
        self._alt_trim        = 0.0     # m，加到 target_alt 的世界系 z 偏调（NED）
        self._alt_trim_frozen = False   # leader 起动后冻结，圆周中不再变
        # 连续 EKF-valid 帧计数,用于 world_birth 校准的收敛门控
        self._valid_streak = [0] * self.num_drones
        # 校准滚动窗口：最近若干帧的 local (x,y,z)，用于稳定性门控
        self._calib_win = [[] for _ in range(self.num_drones)]

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
        self._ocp_ready = True

        # companion 安全滤波层（独立于 MPC，下发前过一道硬保护；不动 OCP→不清缓存）
        self._relinquished = False
        self._relinquish_cooldown_end = 0.0  # monotonic time; prevent rapid re-RELINQUISH
        self._safety_self_timeout = 0.3   # 默认值；safety ON 时由参数覆盖
        if bool(self.get_parameter('safety_filter_enable').value):
            s_track = float(self.get_parameter('safety_max_track_dist').value)
            s_maxa  = float(self.get_parameter('safety_max_alt').value)
            s_mina  = float(self.get_parameter('safety_min_alt').value)
            s_demg  = float(self.get_parameter('safety_d_emergency').value)
            self._safety_self_timeout = float(self.get_parameter('safety_self_timeout').value)
            s_dwarn = max(s_demg + 0.5, d_safe + 0.5)
            self.safety = SafetyFilter(
                max_track_dist=s_track, max_alt=s_maxa, min_alt=s_mina,
                d_emergency=s_demg, d_warn=s_dwarn,
                max_speed=self.max_speed, max_climb=self.max_climb,
                max_accel=self.max_accel, drone_id=self.drone_id)
            self.get_logger().info(
                f'safety_filter ON: track<{s_track}m alt[{s_mina},{s_maxa}]m '
                f'd_emerg={s_demg}m d_warn={s_dwarn}m self_timeout={self._safety_self_timeout}s')
        else:
            self.safety = None
            self.get_logger().warn('safety_filter OFF（仅调试用，真机务必开）')

        # companion 安全滤波层（独立于 MPC，下发前过一道硬保护；不动 OCP→不清缓存）
        self._relinquished = False
        self._relinquish_cooldown_end = 0.0  # monotonic time; prevent rapid re-RELINQUISH
        self._safety_self_timeout = 0.3   # 默认值；safety ON 时由参数覆盖
        if bool(self.get_parameter('safety_filter_enable').value):
            s_track = float(self.get_parameter('safety_max_track_dist').value)
            s_maxa  = float(self.get_parameter('safety_max_alt').value)
            s_mina  = float(self.get_parameter('safety_min_alt').value)
            s_demg  = float(self.get_parameter('safety_d_emergency').value)
            self._safety_self_timeout = float(self.get_parameter('safety_self_timeout').value)
            s_dwarn = max(s_demg + 0.5, d_safe + 0.5)
            self.safety = SafetyFilter(
                max_track_dist=s_track, max_alt=s_maxa, min_alt=s_mina,
                d_emergency=s_demg, d_warn=s_dwarn,
                max_speed=self.max_speed, max_climb=self.max_climb,
                max_accel=self.max_accel, drone_id=self.drone_id)
            self.get_logger().info(
                f'safety_filter ON: track<{s_track}m alt[{s_mina},{s_maxa}]m '
                f'd_emerg={s_demg}m d_warn={s_dwarn}m self_timeout={self._safety_self_timeout}s')
        else:
            self.safety = None
            self.get_logger().warn('safety_filter OFF（仅调试用，真机务必开）')

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
        # 控制定时器与所有订阅分属不同回调组 + MultiThreadedExecutor：
        # 否则单线程 executor 下 50Hz 控制定时器会被 9 邻居+leader+位置订阅的回调洪流
        # 偶发拖过 0.5s → offboard setpoint 断流 → PX4 报 offboard_control_signal_lost → Hold。
        # 共享态(ds.pos/vel、leader_pos、peer_predictions)均为原子 rebind，跨线程读最多取到旧一帧，安全。
        self._cb_control = MutuallyExclusiveCallbackGroup()
        self._cb_subs = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            VehicleStatus,
            topic_for_drone(self.drone_id, 'out/vehicle_status'),
            self._on_vehicle_status, qos, callback_group=self._cb_subs,
        )
        self.create_subscription(
            VehicleAttitude,
            topic_for_drone(self.drone_id, 'out/vehicle_attitude'),
            self.on_self_attitude, qos, callback_group=self._cb_subs,
        )
        self.create_subscription(
            VehicleLocalPosition,
            topic_for_drone(self.drone_id, 'out/vehicle_local_position'),
            self._make_pos_callback(self.drone_id), qos, callback_group=self._cb_subs,
        )
        for j in self.neighbours:
            self.create_subscription(
                VehicleLocalPosition,
                topic_for_drone(j, 'out/vehicle_local_position'),
                self._make_pos_callback(j), qos, callback_group=self._cb_subs,
            )
        leader_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            Float64MultiArray, '/leader/state', self.on_leader_state, leader_qos,
            callback_group=self._cb_subs,
        )
        # Tier2: 全员高度基准 —— drone0 广播当前 ref_alt，各机据此 re-sync world_birth_z
        datum_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_alt_datum = (
            self.create_publisher(Float64, '/swarm/alt_datum', datum_qos)
            if self.drone_id == 0 else None
        )
        self.create_subscription(
            Float64, '/swarm/alt_datum', self._on_alt_datum, datum_qos,
            callback_group=self._cb_subs,
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
                self._make_pred_callback(j), 10, callback_group=self._cb_subs,
            )
        # 控制定时器单独成组 → 与订阅并行、永不被回调洪流饿死
        self.timer = self.create_timer(
            1.0 / self.control_hz, self.control_loop,
            callback_group=self._cb_control,
        )

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

            # Tier2: 持续跟踪当前(滤波)ref_alt（标定后 ref_alt 仍会温漂）
            if math.isfinite(msg.ref_alt) and msg.ref_alt > 0.0:
                if self._ref_alt_now[drone_idx] is None:
                    self._ref_alt_now[drone_idx] = float(msg.ref_alt)
                else:
                    a = self._alt_ref_alpha
                    self._ref_alt_now[drone_idx] = (
                        (1.0 - a) * self._ref_alt_now[drone_idx] + a * float(msg.ref_alt))

            # --- detect EKF reset; update dynamic birth offset ---
            # Skip until first position calibrated world_birth — stale resets
            # from before MPC startup would corrupt the offset.
            _EKF_RESET_CAP = 5.0  # m — ignore implausibly large SITL startup glitches
            if self._pos_calibrated[drone_idx]:
                if msg.xy_reset_counter > self._prev_xy_reset[drone_idx]:
                    dx, dy = float(msg.delta_xy[0]), float(msg.delta_xy[1])
                    reset_mag = math.hypot(dx, dy)
                    if reset_mag > _EKF_RESET_CAP:
                        self.get_logger().error(
                            f'[veh {drone_idx}] xy reset #{msg.xy_reset_counter} '
                            f'IGNORED (mag={reset_mag:.1f}m > cap={_EKF_RESET_CAP}m); '
                            f'world_birth unchanged'
                        )
                    else:
                        self.world_birth[drone_idx, 0] -= dx
                        self.world_birth[drone_idx, 1] -= dy
                        self.get_logger().warn(
                            f'[veh {drone_idx}] xy reset #{msg.xy_reset_counter}, '
                            f'delta=({dx:+.2f}, {dy:+.2f}); '
                            f'world_birth -> ({self.world_birth[drone_idx,0]:+.2f}, '
                            f'{self.world_birth[drone_idx,1]:+.2f})'
                        )
                    self._prev_xy_reset[drone_idx] = msg.xy_reset_counter
                # Z reset：补偿 world_birth_z。
                # z_reset 改变 local_z 与 GPS 海拔的对应关系。由于 world_birth_z
                # 已用 GPS 海拔差校准，z_reset 后必须同步补偿，否则 MPC 世界坐标系
                # 中 ds.pos 会跳变，导致各机高度不一致。
                if msg.z_reset_counter > self._prev_z_reset[drone_idx]:
                    # 起飞期(离地<1m)的 z_reset 多为 EKF 沉降/气压计settle，把跳变补进
                    # world_birth_z 会将一次性偏移永久烤成真高误差(d5 起飞期 reset#3
                    # delta=-0.76 → 真高永久偏 +0.76m)。与 alt_resync 离地门控(:843)同
                    # 思路：近地 reset 只推进计数器、不补偿；飞行中 reset 仍补偿以保世界系
                    # 连续、不扰编队几何。msg 为 drone_idx 本机 local_position，-z=离地高度。
                    alt_agl = -float(msg.z)
                    if alt_agl < 1.0:
                        self.get_logger().warn(
                            f'[veh {drone_idx}] z reset #{msg.z_reset_counter}, '
                            f'delta={msg.delta_z:+.2f}; 离地{alt_agl:.2f}m<1m(起飞期)'
                            f'→ 不补偿 world_birth_z(保持 {self.world_birth[drone_idx,2]:+.2f})'
                        )
                    else:
                        self.world_birth[drone_idx, 2] -= float(msg.delta_z)
                        self.get_logger().warn(
                            f'[veh {drone_idx}] z reset #{msg.z_reset_counter}, '
                            f'delta={msg.delta_z:+.2f}; '
                            f'world_birth_z → {self.world_birth[drone_idx,2]:+.2f}'
                        )
                    self._prev_z_reset[drone_idx] = msg.z_reset_counter

            # --- one-time world_birth calibration, GATED on EKF convergence ---
            # 上电瞬间 PX4 就会发 vehicle_local_position,但此时 EKF 还没 GPS fix,
            # x/y 是几十米级瞬态值。若直接拿首帧校准,会把垃圾烤进 world_birth,
            # 造成机间坐标系不一致(悬停被各机"守出生点"掩盖,line 模式才暴露)。
            # 因此必须等 xy_valid/z_valid 且连续稳定 calib_settle_frames 帧再锁定。
            if not self._pos_calibrated[drone_idx]:
                pos_ok = (msg.xy_valid and msg.z_valid
                          and math.isfinite(msg.x)
                          and math.isfinite(msg.y)
                          and math.isfinite(msg.z))
                if not pos_ok:
                    self._valid_streak[drone_idx] = 0
                    self._calib_win[drone_idx].clear()
                    return
                self._valid_streak[drone_idx] += 1

                # 滚动窗口：不数绝对值，只看 local 是否已"停止漂移"
                win = self._calib_win[drone_idx]
                win.append((msg.x, msg.y, msg.z))
                if len(win) > self.calib_stable_window:
                    win.pop(0)

                arr = np.array(win)
                spread = (arr.max(0) - arr.min(0)) if len(win) >= 2 else np.full(3, np.inf)
                # 用收敛后的窗口均值当基准(抹平单帧噪声),而非抖动单帧
                first_local = arr.mean(0)
                stable = (self._valid_streak[drone_idx] >= self.calib_settle_frames
                          and len(win) >= self.calib_stable_window
                          and float(spread.max()) < self.calib_stable_tol)
                timed_out = self._valid_streak[drone_idx] >= self.calib_timeout_frames

                # 绝对门控：EKF 收敛后静止时 |local_xy|≈0；仍几十米=GPS fix 前暂态，
                # 绝不烤进 world_birth(否则机间世界系不一致→编队塌缩)。timeout 也不放行，
                # 只升级为告警；未校准时各机保持出生点悬停(天然间距)，安全可查。
                near_origin = float(np.linalg.norm(first_local[:2])) < self.calib_max_origin_offset
                if not near_origin:
                    if timed_out and self._valid_streak[drone_idx] % 50 == 0:
                        self.get_logger().error(
                            f'[veh {drone_idx}] calib STUCK: |local_xy|='
                            f'{float(np.linalg.norm(first_local[:2])):.1f}m 远离原点'
                            f'(EKF 未 fix?)，拒绝校准→保持出生点悬停'
                        )
                    return
                if not (stable or timed_out):
                    return

                # 锁定时抓 ref_alt(此刻 EKF 已收敛/GPS fix,避免冻结 fix 前暂态值)
                if hasattr(msg, 'ref_alt') and math.isfinite(msg.ref_alt) and msg.ref_alt > 0:
                    self._ref_alt[drone_idx] = float(msg.ref_alt)
                self.world_birth[drone_idx] = (
                    self.birth_positions[drone_idx] - first_local
                )
                # Z 校准：用 EKF 参考海拔差异补偿各机 home 海拔不同。
                # 各机 EKF 启动时气压计读数不同 → home 海拔(ref_alt)不同
                # → 同一 local_z 对应不同实际海拔。
                # 用 ref_alt 差作为 world_birth_z 偏移量，
                # 使 MPC 世界坐标系中所有机共享同一海拔基准。
                ref_0   = self._ref_alt[0]
                ref_i   = self._ref_alt[drone_idx]
                if self._alt_sync_enable and ref_0 is not None and ref_i is not None:
                    alt_offset = ref_0 - ref_i   # 正值 = 我比基准(home)低
                    self.world_birth[drone_idx, 2] = (
                        self.birth_positions[drone_idx, 2] + alt_offset
                    )
                    self.get_logger().info(
                        f'[veh {drone_idx}] alt sync: '
                        f'my_ref_alt={ref_i:.2f} ref_alt_0={ref_0:.2f} '
                        f'offset={alt_offset:+.2f}m → '
                        f'world_birth_z={self.world_birth[drone_idx,2]:.2f}'
                    )
                else:
                    # alt_sync 关闭(SITL同地面)或 ref_alt 不可用 → 用 birth_z，各机各控离地 target_alt
                    self.world_birth[drone_idx, 2] = self.birth_positions[drone_idx, 2]
                    self.get_logger().info(
                        f'[veh {drone_idx}] alt_sync {"disabled" if not self._alt_sync_enable else "no ref_alt"}, '
                        f'using birth_z={self.birth_positions[drone_idx,2]:.2f}'
                    )
                self._prev_xy_reset[drone_idx] = msg.xy_reset_counter
                self._prev_z_reset[drone_idx] = msg.z_reset_counter
                self._pos_calibrated[drone_idx] = True
                tag = 'STABLE' if stable else 'TIMEOUT(not converged)'
                self.get_logger().info(
                    f'first position from drone {drone_idx} '
                    f'(calibrated [{tag}] after {self._valid_streak[drone_idx]} frames, '
                    f'spread=({spread[0]:.3f},{spread[1]:.3f},{spread[2]:.3f})): '
                    f'local_mean=({first_local[0]:.2f}, {first_local[1]:.2f}, {first_local[2]:.2f}) '
                    f'world_birth=({self.world_birth[drone_idx,0]:.2f}, '
                    f'{self.world_birth[drone_idx,1]:.2f}, '
                    f'{self.world_birth[drone_idx,2]:.2f})'
                )

            ds.received = True
            ds.last_stamp = now
            ds.xy_valid = bool(msg.xy_valid)   # EKF 健康（safety_filter 估计门）
            ds.z_valid = bool(msg.z_valid)
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
        """Retry ARM + OFFBOARD until confirmed. Call every ~100 frames (2 s at 50 Hz).
        auto_arm_enable=false（真机）时不发指令，只被动等待飞手 RC 操作后的状态确认。"""
        if self._arm_offboard_confirmed:
            return
        if self.auto_arm_enable:
            if self._nav_state != 14:
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            if self._arming_state != 2:
                self._send_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        if self._nav_state == 14 and self._arming_state == 2:
            self._arm_offboard_confirmed = True
            if self._relinquished:
                self._relinquished = False
                self.get_logger().info(
                    f'drone {self.drone_id}: OFFBOARD + ARMED confirmed (recovered from RELINQUISH)')
            else:
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
        # 加速度（圆周运动向心加速度），向后兼容旧版 leader_node
        if len(msg.data) >= 10:
            ax = float(msg.data[8]) if math.isfinite(msg.data[8]) else 0.0
            ay = float(msg.data[9]) if math.isfinite(msg.data[9]) else 0.0
            self.leader_acc = np.array([ax, ay, 0.0])
        else:
            self.leader_acc = np.zeros(3)

    def _make_pred_callback(self, drone_idx):
        def cb(msg):
            data = np.asarray(msg.data, dtype=float)
            if data.size % 3 != 0 or data.size == 0:
                return
            # P2 注入: 丢包 — 直接丢弃本条预测（超时后走既有失联降级路径）
            if self.comms_dropout > 0.0 and random.random() < self.comms_dropout:
                return
            traj = data.reshape(-1, 3)
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.comms_delay_s > 0.0:
                # P2 注入: 延迟 — 入缓冲，到期才可见；stamp 记原始到达时刻，
                # 使既有 latency 时间对齐自然吸收注入延迟
                self._pred_delay_buf.setdefault(drone_idx, []).append((now, traj))
            else:
                self.peer_predictions[drone_idx] = traj
                self.peer_prediction_stamps[drone_idx] = now
        return cb

    def _drain_pred_delay_buf(self, now):
        """P2: 把注入延迟已到期的邻居预测放行到 peer_predictions。"""
        if self.comms_delay_s <= 0.0:
            return
        for j, buf in self._pred_delay_buf.items():
            while buf and (now - buf[0][0]) >= self.comms_delay_s:
                arrival_t, traj = buf.pop(0)
                self.peer_predictions[j] = traj
                self.peer_prediction_stamps[j] = arrival_t

    # =================================================================
    # control loop
    # =================================================================
    def _on_alt_datum(self, msg):
        self._datum_ref_alt = float(msg.data)

    def _eff_target_alt(self):
        """有效目标高度（世界系 NED）= 基准 target_alt + 悬停期标定的 alt_trim。
        alt_trim=0 时等价原行为。"""
        return self.target_alt + self._alt_trim

    def _update_alt_trim(self):
        """悬停期：用本机 ref_alt 与 drone0 datum 之差偏调 target_alt，使各机收敛到
        同一绝对海拔（=同真高）。leader 起动即冻结，圆周中不再变。只调设定点、不动
        world_birth_z，故不扰碰撞/编队几何（避开 alt_resync 撞机根因）。"""
        if not self._alt_trim_enable or self._alt_trim_frozen:
            return
        # leader 起动 → 冻结当前 trim
        if float(np.linalg.norm(self.leader_vel)) > 0.05:
            self._alt_trim_frozen = True
            self.get_logger().info(
                f'[d{self.drone_id}] alt_trim 冻结 @ {self._alt_trim:+.2f}m（leader 起动）')
            return
        my_ref = self._ref_alt_now[self.drone_id]
        if (self._datum_ref_alt is None or my_ref is None
                or not self._pos_calibrated[self.drone_id]):
            return
        # 离地<1m（起飞期）不调，防地面暂态
        self_ds = self.drone_states[self.drone_id]
        if not self_ds.received or (-self_ds.pos[2]) < 1.0:
            return
        raw = float(np.clip(my_ref - self._datum_ref_alt,
                            -self._alt_trim_max, self._alt_trim_max))
        # 低通收敛，防 ref_alt 噪声抖动
        self._alt_trim = 0.9 * self._alt_trim + 0.1 * raw

    def _resync_world_birth_z(self, dt):
        """Tier2: 把(已标定)机的 world_birth_z 限速拉向 birth_z + (datum - 当前ref_alt)。
        ref_alt 连续温漂不触发 z_reset，:507 补不到；这里持续纠偏，且限速使 MPC 只看到
        ≤alt_resync_rate 的 z 速度，不会跳。基准 = drone0 当前 ref_alt（广播给全员）。"""
        # drone0 广播当前基准（己方标定后才发，避免广播 GPS-fix 前暂态）
        if (self.drone_id == 0 and self._pos_calibrated[0]
                and self._ref_alt_now[0] is not None):
            self._datum_ref_alt = self._ref_alt_now[0]
            if self.pub_alt_datum is not None:
                m = Float64()
                m.data = float(self._datum_ref_alt)
                self.pub_alt_datum.publish(m)
        if not self._alt_resync_enable or self._datum_ref_alt is None:
            return
        # 自机离地 < 1m 时不做 resync：防起飞期 world_birth_z 误漂导致安全层误判高度
        self_ds_alt = self.drone_states[self.drone_id]
        if not self_ds_alt.received or (-self_ds_alt.pos[2]) < 1.0:
            return
        step = max(0.0, self._alt_resync_rate * dt)
        for idx in [self.drone_id] + list(self.neighbours):
            if not self._pos_calibrated[idx] or self._ref_alt_now[idx] is None:
                continue
            birth_z = float(self.birth_positions[idx, 2])
            target  = birth_z + (self._datum_ref_alt - self._ref_alt_now[idx])
            if abs(target - birth_z) > self._alt_resync_max:
                continue   # ref_alt 异常大偏移，安全起见不跟
            cur = float(self.world_birth[idx, 2])
            err = target - cur
            self.world_birth[idx, 2] = (
                target if abs(err) <= step else cur + math.copysign(step, err))

    def control_loop(self):
        # RELINQUISH 后不停循环：让 line ~935 的 OFFBOARD 丢失恢复机制
        # 检测 nav!=14 → 重新 ARM+OFFBOARD → 恢复控制。
        # 不再 return（旧行为导致 RELINQUISH 后永卡 Hold）。
        self.publish_offboard_mode()
        now_ros = self.get_clock().now()
        dt = (now_ros - self.last_control_time).nanoseconds * 1e-9
        self.last_control_time = now_ros
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / self.control_hz

        self._resync_world_birth_z(dt)   # Tier2: 补偿 ref_alt 连续温漂
        self._update_alt_trim()          # 悬停期高度 trim（leader 起动前标定，之后冻结）

        # 使用当前姿态 yaw，避免起飞时强制转向正北
        yaw_hold = self.attitude_yaw if self.attitude_received else 0.0

        if self._startup_counter < self.startup_zero_vel_frames:
            self._startup_counter += 1
            self._hover_active = True
            self.publish_position_setpoint(self._hover_setpoint_world(), np.zeros(3), yaw_hold)
            return

        # After startup: retry ARM+OFFBOARD every 50 frames (1 s) until confirmed.
        # Also re-arm if OFFBOARD was lost after initial confirmation (nav dropped from 14).
        if self._arm_offboard_confirmed and self.auto_arm_enable and self._nav_state != 14:
            self._arm_offboard_confirmed = False
            self._cmd_retry_counter = 0
            self.get_logger().warn(
                f'drone {self.drone_id}: OFFBOARD LOST (nav={self._nav_state}) — resetting, will re-arm')
        if not self._arm_offboard_confirmed:
            if not self._ocp_ready:   # 等 acados 编译完再尝试 ARM，避免 OFFBOARD 超时
                self._hover_active = True
                self.publish_position_setpoint(self._hover_setpoint_world(), np.zeros(3), yaw_hold)
                return
            self._cmd_retry_counter += 1
            if self._cmd_retry_counter % 50 == 1:
                self._arm_and_engage_offboard()
                if not self._arm_offboard_confirmed:
                    action = ('retry ARM+OFFBOARD' if self.auto_arm_enable
                              else 'waiting RC ARM+OFFBOARD')
                    self.get_logger().info(
                        f'drone {self.drone_id}: {action} '
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
            wb = self.world_birth[self.drone_id]
            local_now = self_ds.pos - wb     # = PX4 原始 local（校准的逆运算）
            self.get_logger().info(
                f'[d{self.drone_id}] solve: status={info["status"]} '
                f'time={self._last_solve_ms:.2f}ms cost={info["cost"]:.1f} '
                f'pos=({self_ds.pos[0]:.2f},{self_ds.pos[1]:.2f},{self_ds.pos[2]:.2f}) '
                f'local=({local_now[0]:.2f},{local_now[1]:.2f},{local_now[2]:.2f}) '
                f'wbirth=({wb[0]:.2f},{wb[1]:.2f},{wb[2]:.2f}) '
                f'ref=({x_ref[0,0]:.2f},{x_ref[0,1]:.2f}) '
                f'leader=({self.leader_pos[0]:.2f},{self.leader_pos[1]:.2f}) '
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
        # Use predicted velocity at k=1 as velocity setpoint base.
        # z-axis: pure P-controller for altitude hold (decoupled from XY).
        pred_vel = x_pred[1, 3:6].copy()

        # Position error → velocity correction (P-controller)
        ref_pos = x_ref[0, 0:3]
        pos_err_vec = ref_pos - self_ds.pos
        Kp_pos = 1.0  # position gain (m/s per m of error)
        vel_correction = np.clip(pos_err_vec * Kp_pos, -self.max_speed, self.max_speed)

        # Blend: MPC velocity + position correction
        # 横向：MPC 优化速度前馈 + 候选①可选的饱和限幅位置 P。原先叠加 0.5·Kp·pos_err
        # 的外层「无限幅」比例环 → 僚机过冲、圆周震荡(1c4bc5a 删之)。这里改为有界 P：
        # vel_xy_kp=0 时退回纯前馈(等价 1c4bc5a)；>0 时纠偏被 vel_xy_cap 限幅，
        # 抓住密集 grid 运动中碰撞约束推离槽位的漂移、防发散，又不致无阻尼震荡。
        # 垂直：保留纯 P(高度保持，与横向解耦，不引起震荡)。
        vel_sp = np.zeros(3)
        vel_sp[0] = pred_vel[0]
        vel_sp[1] = pred_vel[1]
        if self.vel_xy_kp > 0.0:
            corr_xy = np.clip(pos_err_vec[:2] * self.vel_xy_kp,
                              -self.vel_xy_cap, self.vel_xy_cap)
            vel_sp[0] += corr_xy[0]
            vel_sp[1] += corr_xy[1]
        vel_sp[2] = vel_correction[2]  # pure P-controller for z (altitude hold)

        # Clip to limits
        vel_xy_norm = float(np.linalg.norm(vel_sp[:2]))
        if vel_xy_norm > self.max_speed:
            vel_sp[:2] *= self.max_speed / vel_xy_norm
        vel_sp[2] = np.clip(vel_sp[2], -self.max_climb, self.max_climb)

        # Safety: NaN check
        if not np.all(np.isfinite(vel_sp)):
            vel_sp = np.zeros(3)

        # ── companion 安全滤波层：围栏/碰撞地板/估计门/失效状态机（下发前）──
        if self.safety is not None:
            now_s = now_ros.nanoseconds * 1e-9
            nbrs = [(self.drone_states[j].pos,
                     self.drone_states[j].received
                     and (now_s - self.drone_states[j].last_stamp) <= self.neighbour_timeout)
                    for j in self.neighbours]
            est_ok = (self_ds.xy_valid and self_ds.z_valid
                      and (now_s - self_ds.last_stamp) <= self._safety_self_timeout)
            sres = self.safety.step(self_ds.pos, self_ds.vel, vel_sp, ref_pos, dt,
                                    neighbours=nbrs, est_ok=est_ok)
            vel_sp = sres['vel_sp']
            if sres['state'] != 'NORMAL':
                self._hover_active = True
                if self._dbg_counter % int(self.control_hz) == 0:
                    self.get_logger().warn(
                        f"[d{self.drone_id}] SAFETY {sres['state']} {sres['reasons']}")
            if not sres['publish']:
                now_s = now_ros.nanoseconds * 1e-9
                if now_s < self._relinquish_cooldown_end:
                    # 冷却期内：允许 MPC 继续发 setpoint（PX4 在 OFFBOARD → 不会失控）
                    # 同时让 OFFBOARD 丢失恢复机制（line ~935）重新 ARM
                    pass
                else:
                    # RELINQUISH：停发，交还 PX4 失效保护（COM_OF_LOSS_T→COM_OBL_RC_ACT）
                    self._relinquished = True
                    self._relinquish_cooldown_end = now_s + 5.0   # 5s 冷却，防反复触发
                    self.get_logger().error(
                        f"[d{self.drone_id}] SAFETY RELINQUISH {sres['reasons']} — 交还 PX4, "
                        f"cooldown 5s")
                    self._publish_health()
                    return

        # Position error for logging
        pos_err = float(np.linalg.norm(pos_err_vec[:2]))
        self._last_pos_err = pos_err
        self._last_z_err = abs(float(self_ds.pos[2] - self._eff_target_alt()))

        # ── 诊断：横向跟踪误差分解为 radial(径向)/tangential(切向)，1Hz ──
        # 切向 = leader 速度方向；径向 = 向心加速度反向(指向圆外)；圆周时二者正交。
        #   e_tan > 0 → 参考在前方，僚机滞后(切向滞后/相位问题)
        #   e_rad > 0 → 参考更靠外，僚机切内圈(径向/曲率/增益问题)
        # 看震荡主要落在哪个分量，即可定位机理(切向=速度前馈/相位；径向=曲率/增益)。
        if self._dbg_counter % int(self.control_hz) == 0:
            v_xy = self.leader_vel[:2]
            a_xy = self.leader_acc[:2]
            v_norm = float(np.linalg.norm(v_xy))
            a_norm = float(np.linalg.norm(a_xy))
            if v_norm > 1e-3 and a_norm > 1e-3:
                t_hat = v_xy / v_norm        # 切向(运动方向)
                r_hat = -a_xy / a_norm       # 径向向外(向心反向)
                e_tan = float(np.dot(pos_err_vec[:2], t_hat))
                e_rad = float(np.dot(pos_err_vec[:2], r_hat))
                self.get_logger().info(
                    f'[d{self.drone_id}] track-err total={pos_err:.3f}m '
                    f'radial={e_rad:+.3f} tangential={e_tan:+.3f} '
                    f'(|v|={v_norm:.2f} |a|={a_norm:.2f})'
                )

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
        leader_acc_safe = (self.leader_acc.copy()
                           if np.all(np.isfinite(self.leader_acc)) else np.zeros(3))
        for k in range(N + 1):
            t = k * self.mpc_dt
            # 二阶预测：pos + vel*t + 0.5*acc*t²（圆周运动时捕捉向心加速度）
            pos = leader_pos_safe + leader_vel_safe * t + 0.5 * leader_acc_safe * t * t + self.my_offset
            pos[2] = self._eff_target_alt()
            x_ref[k, 0:3] = pos
            # 速度前馈也随加速度变化：vel(t) = vel(0) + acc*t
            vel_t = leader_vel_safe[:2] + leader_acc_safe[:2] * t
            x_ref[k, 3:5] = vel_t
            x_ref[k, 5]   = 0.0
        return x_ref

    def _collect_neighbour_predictions(self):
        if not self.neighbours:
            return None
        now = self.get_clock().now().nanoseconds * 1e-9
        self._drain_pred_delay_buf(now)
        N1 = self.N + 1
        out = np.zeros((len(self.neighbours), N1, 3))
        for idx, j in enumerate(self.neighbours):
            traj = self.peer_predictions.get(j)
            stamp = self.peer_prediction_stamps.get(j, 0.0)
            pred_fresh = (traj is not None and traj.shape[0] >= 1
                          and (now - stamp) <= self.neighbour_timeout)
            if pred_fresh:
                # 时间对齐：平移掉通信延迟对应的步数
                latency = now - stamp
                shift = min(int(round(latency / self.mpc_dt)), traj.shape[0] - 1)
                traj = traj[shift:]
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
            float(self._last_z_err),   # index 6: 高度误差 |z - target_alt| (m)
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
            return np.array([ds.pos[0], ds.pos[1], self._eff_target_alt()])
        birth = self.world_birth[self.drone_id]
        return np.array([birth[0], birth[1], self._eff_target_alt()])

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
    # 多线程 executor：控制定时器(自己的回调组)与订阅(另一组)并行，
    # 保证 50Hz offboard 心跳/ setpoint 不被订阅回调拖延 → 不丢 offboard 信号。
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()