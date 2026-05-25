#!/usr/bin/env python3
"""
虚拟领队节点：发布 /leader/state (Float32MultiArray)
格式: [time, x, y, z, vx, vy, vz, yaw]

支持三种运动模式（通过 ROS2 参数配置）:
  hover  — 悬停在固定点（默认）
  circle — 匀速圆周运动
  line   — 沿 X 轴匀速直线飞行
"""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class LeaderNode(Node):
    def __init__(self):
        super().__init__('leader_node')

        self.declare_parameter('mode',       'hover')   # hover | circle | line
        self.declare_parameter('start_x',     0.0)
        self.declare_parameter('start_y',     0.0)
        self.declare_parameter('altitude',   -5.0)      # NED：-5 = 离地5m
        self.declare_parameter('speed',       1.0)      # m/s
        self.declare_parameter('radius',     10.0)      # circle 半径 m
        self.declare_parameter('publish_hz', 50.0)

        self._mode   = str(self.get_parameter('mode').value)
        self._x0     = float(self.get_parameter('start_x').value)
        self._y0     = float(self.get_parameter('start_y').value)
        self._alt    = float(self.get_parameter('altitude').value)
        self._speed  = float(self.get_parameter('speed').value)
        self._radius = float(self.get_parameter('radius').value)
        hz           = float(self.get_parameter('publish_hz').value)

        self._t  = 0.0
        self._dt = 1.0 / hz

        self._pub = self.create_publisher(Float32MultiArray, '/leader/state', 10)
        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f'Leader ready: mode={self._mode}, alt={self._alt}m (NED), '
            f'speed={self._speed}m/s, radius={self._radius}m'
        )

    def _tick(self):
        t = self._t
        self._t += self._dt

        if self._mode == 'circle':
            omega = self._speed / max(self._radius, 0.1)
            x   =  self._x0 + self._radius * math.cos(omega * t)
            y   =  self._y0 + self._radius * math.sin(omega * t)
            vx  = -self._radius * omega * math.sin(omega * t)
            vy  =  self._radius * omega * math.cos(omega * t)
            yaw =  math.atan2(vy, vx)
        elif self._mode == 'line':
            x   = self._x0 + self._speed * t
            y   = self._y0
            vx  = self._speed
            vy  = 0.0
            yaw = 0.0
        else:   # hover
            x, y   = self._x0, self._y0
            vx, vy = 0.0, 0.0
            yaw    = 0.0

        msg = Float32MultiArray()
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
