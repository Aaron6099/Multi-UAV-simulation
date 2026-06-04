#!/usr/bin/env python3
"""
虚拟领队节点：发布 /leader/state (Float64MultiArray)
格式: [time, x, y, z, vx, vy, vz, yaw]

支持三种运动模式（通过 ROS2 参数配置）:
  hover  — 悬停在固定点（默认）
  circle — 匀速圆周运动
  line   — 沿 X 轴匀速直线飞行

yaw_mode 参数（运行时可切换）:
  fixed   — 固定起飞朝向（默认）
  center  — 朝向圆心（摄影/观测）
  tangent — 跟随飞行方向（仿生/展示）

运行时切换:
  ros2 param set /leader_node yaw_mode fixed
  ros2 param set /leader_node yaw_mode center
  ros2 param set /leader_node yaw_mode tangent
"""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32MultiArray

# yaw 变化率上限 (rad/s)，防止机体抖动
MAX_YAW_RATE = math.radians(45.0)  # 45°/s


def _wrap_angle(a):
    """将角度归一化到 [-pi, pi]。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _limit_yaw_rate(current, target, max_rate, dt):
    """限幅 yaw 变化率，返回平滑过渡后的 yaw。"""
    diff = _wrap_angle(target - current)
    max_step = max_rate * dt
    if abs(diff) <= max_step:
        return target
    return current + max_step * (1.0 if diff > 0 else -1.0)


class LeaderNode(Node):
    def __init__(self):
        super().__init__('leader_node')

        self.declare_parameter('mode',       'hover')   # hover | circle | line
        self.declare_parameter('yaw_mode',   'fixed')   # fixed | center | tangent
        self.declare_parameter('start_x',     0.0)
        self.declare_parameter('start_y',     0.0)
        self.declare_parameter('altitude',   -5.0)      # NED：-5 = 离地5m
        self.declare_parameter('speed',       1.0)      # m/s
        self.declare_parameter('radius',     10.0)      # circle 半径 m
        self.declare_parameter('publish_hz', 50.0)
        self.declare_parameter('max_distance', 20.0)  # 直线模式最大飞行距离 (m)，到达后悬停
        self.declare_parameter('line_decel',   0.5)   # 直线终点前减速度 (m/s²)，平滑刹停防僚机过冲
        self.declare_parameter('start_delay', 30.0)   # 起飞等待 (s)：leader 先原地不动，等僚机 ARM+爬升+组队（10s 太短，launch 默认同步为 30）
        # 闭环就绪门控：等各机进编队(pos_err<阈值)再开始运动，替代死等固定 start_delay
        self.declare_parameter('num_drones',        1)
        self.declare_parameter('ready_gate_enable', True)
        self.declare_parameter('ready_pos_err',     0.5)   # m，进编队判定阈值
        self.declare_parameter('ready_hold',        2.0)   # s，就绪需连续保持时长
        self.declare_parameter('ready_timeout',     90.0)  # s，超时兜底：仍未就绪也开动(告警)
        self.declare_parameter('health_timeout',    2.0)   # s，health 超此视为失联

        self._mode   = str(self.get_parameter('mode').value)
        self._x0     = float(self.get_parameter('start_x').value)
        self._y0     = float(self.get_parameter('start_y').value)
        self._alt    = float(self.get_parameter('altitude').value)
        self._speed  = float(self.get_parameter('speed').value)
        self._radius = float(self.get_parameter('radius').value)
        hz           = float(self.get_parameter('publish_hz').value)

        self._t  = 0.0
        self._dt = 1.0 / hz
        self._max_distance = float(self.get_parameter('max_distance').value)
        self._line_decel   = max(1e-3, float(self.get_parameter('line_decel').value))
        self._start_delay = float(self.get_parameter('start_delay').value)
        self._initial_yaw = None   # 首次发布时记录
        self._current_yaw = 0.0    # 当前平滑后的 yaw（用于限幅）

        # line 模式：对准阶段
        self._line_align_done = False
        self._line_start_t = 0.0    # 对准完成、开始平移时的运动时钟
        self._hold_logged = False   # 起飞等待提示只打一次

        # 就绪门控状态
        self._num_drones        = int(self.get_parameter('num_drones').value)
        self._ready_gate_enable = bool(self.get_parameter('ready_gate_enable').value)
        self._ready_pos_err     = float(self.get_parameter('ready_pos_err').value)
        self._ready_hold        = float(self.get_parameter('ready_hold').value)
        self._ready_timeout     = float(self.get_parameter('ready_timeout').value)
        self._health_timeout    = float(self.get_parameter('health_timeout').value)
        self._health_pos_err = [None] * max(1, self._num_drones)
        self._health_stamp   = [None] * max(1, self._num_drones)
        self._motion_started = False
        self._motion_start_t = 0.0
        self._ready_since    = None
        self._timeout_warned = False

        self._pub = self.create_publisher(Float64MultiArray, '/leader/state', 10)
        # 订阅各机 MPC health（pos_err=data[5]）用于就绪门控
        for i in range(self._num_drones):
            topic = '/mpc/health' if i == 0 else f'/px4_{i}/mpc/health'
            self.create_subscription(
                Float32MultiArray, topic, self._make_health_cb(i), 10)
        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f'Leader ready: mode={self._mode}, yaw_mode={self.get_parameter("yaw_mode").value}, '
            f'alt={self._alt}m (NED), speed={self._speed}m/s, radius={self._radius}m'
        )

    def _compute_raw_yaw(self, x, y, vx, vy):
        """根据 yaw_mode 计算目标偏航角（无限幅）。"""
        yaw_mode = str(self.get_parameter('yaw_mode').value)

        if yaw_mode == 'center':
            cx = self._x0
            cy = self._y0
            return math.atan2(cy - y, cx - x)
        elif yaw_mode == 'tangent':
            if abs(vx) < 1e-6 and abs(vy) < 1e-6:
                return self._current_yaw
            return math.atan2(vy, vx)
        else:  # fixed
            if self._initial_yaw is None:
                if abs(vx) > 1e-6 or abs(vy) > 1e-6:
                    self._initial_yaw = math.atan2(vy, vx)
                else:
                    self._initial_yaw = 0.0
            return self._initial_yaw

    def _apply_yaw_limits(self, raw_yaw):
        """对目标 yaw 施加变化率限幅，返回平滑后的 yaw。"""
        smoothed = _limit_yaw_rate(self._current_yaw, raw_yaw, MAX_YAW_RATE, self._dt)
        self._current_yaw = smoothed
        return smoothed

    def _make_health_cb(self, idx):
        def cb(msg):
            if len(msg.data) >= 6:
                self._health_pos_err[idx] = float(msg.data[5])
                self._health_stamp[idx] = self.get_clock().now().nanoseconds * 1e-9
        return cb

    def _all_formed_up(self):
        """所有机 health 新鲜且 pos_err < 阈值 → 编队已组好。"""
        if self._num_drones <= 0:
            return False
        now = self.get_clock().now().nanoseconds * 1e-9
        for i in range(self._num_drones):
            stamp = self._health_stamp[i]
            err   = self._health_pos_err[i]
            if stamp is None or (now - stamp) > self._health_timeout:
                return False
            if err is None or err > self._ready_pos_err:
                return False
        return True

    def _should_start_motion(self, t):
        # hover 不动、或门控关闭：退回旧的固定 start_delay 行为
        if self._mode == 'hover' or not self._ready_gate_enable:
            return t >= self._start_delay
        # 闭环就绪：全员组好并连续保持 ready_hold
        if self._all_formed_up():
            if self._ready_since is None:
                self._ready_since = t
                self.get_logger().info(
                    f'all drones formed up (pos_err < {self._ready_pos_err:.2f}m), '
                    f'confirming for {self._ready_hold:.1f}s...')
            elif t - self._ready_since >= self._ready_hold:
                self.get_logger().info(
                    f'formation ready — starting leader motion at t={t:.1f}s')
                return True
        else:
            self._ready_since = None
        # 超时兜底：太久没组好也开动，但大声告警（别静默卡住）
        if t >= self._ready_timeout:
            if not self._timeout_warned:
                self.get_logger().error(
                    f'readiness TIMEOUT at t={t:.1f}s — not all drones formed up; '
                    'starting motion anyway, CHECK STRAGGLERS')
                self._timeout_warned = True
            return True
        return False

    def _tick(self):
        t = self._t
        self._t += self._dt
        # 读取运行时参数（支持 ros2 param set 动态切换）
        self._mode = str(self.get_parameter('mode').value)

        # 运动起始门控：等编队组好(各机 pos_err 小)再开始运动。
        # 死等固定 start_delay 在 5/9 机会猜错(短了僚机边爬边追、长了干等)，
        # 改为订阅各机 health.pos_err 的闭环就绪门控，带超时兜底。
        if not self._motion_started:
            if self._should_start_motion(t):
                self._motion_started = True
                self._motion_start_t = t
            else:
                x, y = self._x0, self._y0
                vx, vy = 0.0, 0.0
                ax, ay = 0.0, 0.0
                yaw = self._apply_yaw_limits(self._compute_raw_yaw(x, y, vx, vy))
                msg = Float64MultiArray()
                msg.data = [float(t), x, y, self._alt, vx, vy, 0.0, yaw, ax, ay]
                self._pub.publish(msg)
                return

        t_move = t - self._motion_start_t

        if self._mode == 'circle':
            omega = self._speed / max(self._radius, 0.1)
            # 圆心在出生点正西，使 t_move=0 时 leader 正好在出生点 (x0,y0)
            # 避免从 hold 切到 circle 时的 10m 位置阶跃
            cx = self._x0 - self._radius
            cy = self._y0
            x   =  cx + self._radius * math.cos(omega * t_move)
            y   =  cy + self._radius * math.sin(omega * t_move)
            vx  = -self._radius * omega * math.sin(omega * t_move)
            vy  =  self._radius * omega * math.cos(omega * t_move)
            # 向心加速度：a = -ω²r，方向指向圆心
            ax  = -omega * vy   # = -ω² * radius * cos(ωt)
            ay  =  omega * vx   # = -ω² * radius * sin(ωt)
            raw_yaw = self._compute_raw_yaw(x, y, vx, vy)
            yaw = self._apply_yaw_limits(raw_yaw)

        elif self._mode == 'line':
            vx = self._speed
            vy = 0.0
            ax, ay = 0.0, 0.0

            # Phase 1: 对准阶段 — 先转 yaw，再开始移动
            if not self._line_align_done:
                x, y = self._x0, self._y0
                target_yaw = self._compute_raw_yaw(x, y, vx, vy)
                yaw = self._apply_yaw_limits(target_yaw)
                if abs(_wrap_angle(yaw - target_yaw)) < math.radians(2.0):
                    self._line_align_done = True
                    self._line_start_t = t_move   # 记录开始平移时刻
                    self.get_logger().info(f'Yaw aligned to {math.degrees(yaw):.1f}°, starting line motion')
                # 对准阶段不移动
                vx, vy = 0.0, 0.0
            else:
                # 梯形速度曲线：巡航 → 终点前按 line_decel 平滑减速到 0。
                # 旧版到点 vx 从 speed 直接跳到 0，速度阶跃致僚机过冲/来回弹几下。
                tau   = t_move - self._line_start_t   # 平移已历时 (s)
                spd   = abs(self._speed)
                sgn   = 1.0 if self._speed >= 0 else -1.0
                s_max = self._max_distance
                a_dec = self._line_decel
                d_brake = min(s_max, spd * spd / (2.0 * a_dec))          # 刹车距离
                t_brake = max(0.0, (s_max - d_brake) / max(spd, 1e-6))   # 开始刹车时刻
                t_decel = spd / a_dec                                    # 减速段时长
                if tau <= t_brake:                    # 巡航
                    d, v, a = spd * tau, spd, 0.0
                elif tau <= t_brake + t_decel:        # 减速到 0
                    td = tau - t_brake
                    d = (s_max - d_brake) + spd * td - 0.5 * a_dec * td * td
                    v = spd - a_dec * td
                    a = -a_dec
                else:                                 # 已停在终点
                    d, v, a = s_max, 0.0, 0.0
                d  = max(0.0, min(d, s_max))
                x  = self._x0 + sgn * d
                y  = self._y0
                vx, vy = sgn * v, 0.0
                ax, ay = sgn * a, 0.0
                raw_yaw = self._compute_raw_yaw(x, y, vx, vy)
                yaw = self._apply_yaw_limits(raw_yaw)

        else:  # hover
            x, y   = self._x0, self._y0
            vx, vy = 0.0, 0.0
            ax, ay = 0.0, 0.0
            raw_yaw = self._compute_raw_yaw(x, y, vx, vy)
            yaw = self._apply_yaw_limits(raw_yaw)

        msg = Float64MultiArray()
        msg.data = [float(t), x, y, self._alt, vx, vy, 0.0, yaw, ax, ay]
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderNode()
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
