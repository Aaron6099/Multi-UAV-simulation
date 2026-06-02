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
from std_msgs.msg import Float64MultiArray

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
        self.declare_parameter('start_delay', 30.0)   # 起飞等待 (s)：leader 先原地不动，等僚机 ARM+爬升+组队（10s 太短，launch 默认同步为 30）

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
        self._start_delay = float(self.get_parameter('start_delay').value)
        self._initial_yaw = None   # 首次发布时记录
        self._current_yaw = 0.0    # 当前平滑后的 yaw（用于限幅）

        # line 模式：对准阶段
        self._line_align_done = False
        self._line_start_t = 0.0    # 对准完成、开始平移时的运动时钟
        self._hold_logged = False   # 起飞等待提示只打一次

        self._pub = self.create_publisher(Float64MultiArray, '/leader/state', 10)
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

    def _tick(self):
        t = self._t
        self._t += self._dt
        # 读取运行时参数（支持 ros2 param set 动态切换）
        self._mode = str(self.get_parameter('mode').value)

        # 起飞等待：前 start_delay 秒 leader 在原点不动，给僚机 ARM+爬升+组队留时间，
        # 避免它们从落后状态边爬边追导致 QP 失败/到点过冲。之后用 t_move 作运动时钟。
        t_move = t - self._start_delay
        if t_move < 0.0:
            if not self._hold_logged:
                self.get_logger().info(
                    f'leader holding at start for {self._start_delay:.1f}s '
                    f'(let drones arm/climb/form up)...')
                self._hold_logged = True
            x, y = self._x0, self._y0
            vx, vy = 0.0, 0.0
            yaw = self._apply_yaw_limits(self._compute_raw_yaw(x, y, vx, vy))
            msg = Float64MultiArray()
            msg.data = [float(t), x, y, self._alt, vx, vy, 0.0, yaw]
            self._pub.publish(msg)
            return

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
            raw_yaw = self._compute_raw_yaw(x, y, vx, vy)
            yaw = self._apply_yaw_limits(raw_yaw)

        elif self._mode == 'line':
            vx = self._speed
            vy = 0.0

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
                x = self._x0 + self._speed * (t_move - self._line_start_t)
                y = self._y0
                # 到达最大距离后悬停
                dist = abs(x - self._x0)
                if dist >= self._max_distance:
                    x = self._x0 + self._max_distance * (1.0 if self._speed > 0 else -1.0)
                    vx, vy = 0.0, 0.0
                raw_yaw = self._compute_raw_yaw(x, y, vx, vy)
                yaw = self._apply_yaw_limits(raw_yaw)

        else:  # hover
            x, y   = self._x0, self._y0
            vx, vy = 0.0, 0.0
            raw_yaw = self._compute_raw_yaw(x, y, vx, vy)
            yaw = self._apply_yaw_limits(raw_yaw)

        msg = Float64MultiArray()
        msg.data = [float(t), x, y, self._alt, vx, vy, 0.0, yaw]
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
