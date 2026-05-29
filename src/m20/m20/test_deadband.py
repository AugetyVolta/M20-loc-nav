#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from collections import deque

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from drdds.msg import NavCmd


class DeadbandTester(Node):
    def __init__(self):
        super().__init__('deadband_tester')

        # ---------------- 参数 ----------------
        self.declare_parameter('cmd_topic', '/NAV_CMD')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('mode', 'yaw')              # x / y / yaw
        self.declare_parameter('start_cmd', 0.25)
        self.declare_parameter('end_cmd', 0.5)
        self.declare_parameter('step_cmd', 0.02)
        self.declare_parameter('hold_sec', 5.0)
        self.declare_parameter('publish_hz', 10.0)

        # 实际运动判定阈值：超过这个值，认为“机器人真的动了”
        self.declare_parameter('feedback_eps_x', 0.03)   # m/s
        self.declare_parameter('feedback_eps_y', 0.03)   # m/s
        self.declare_parameter('feedback_eps_yaw', 0.08) # rad/s

        # 为了降低瞬时噪声，取最近一段时间平均
        self.declare_parameter('avg_window_sec', 0.8)

        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.mode = self.get_parameter('mode').value
        self.start_cmd = float(self.get_parameter('start_cmd').value)
        self.end_cmd = float(self.get_parameter('end_cmd').value)
        self.step_cmd = float(self.get_parameter('step_cmd').value)
        self.hold_sec = float(self.get_parameter('hold_sec').value)
        self.publish_hz = float(self.get_parameter('publish_hz').value)

        self.feedback_eps_x = float(self.get_parameter('feedback_eps_x').value)
        self.feedback_eps_y = float(self.get_parameter('feedback_eps_y').value)
        self.feedback_eps_yaw = float(self.get_parameter('feedback_eps_yaw').value)
        self.avg_window_sec = float(self.get_parameter('avg_window_sec').value)

        if self.mode not in ('x', 'y', 'yaw'):
            raise ValueError("mode 必须是 x / y / yaw")

        # ---------------- 通信 ----------------
        self.pub = self.create_publisher(NavCmd, self.cmd_topic, 10)
        self.sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)

        # ---------------- 状态 ----------------
        self.latest_odom_time = None
        self.feedback_buf = deque()

        self.publish_period = 1.0 / max(self.publish_hz, 1e-3)
        self.timer = self.create_timer(self.publish_period, self.on_timer)

        self.cmd_values = self.build_cmd_list(self.start_cmd, self.end_cmd, self.step_cmd)
        self.phase_idx = 0
        self.phase_start_time = self.get_clock().now().nanoseconds * 1e-9
        self.current_cmd = 0.0
        self.detected_threshold = None

        self.get_logger().warn(
            f"开始测试 deadband: mode={self.mode}, cmd_topic={self.cmd_topic}, "
            f"odom_topic={self.odom_topic}, range=[{self.start_cmd}, {self.end_cmd}], step={self.step_cmd}"
        )
        self.get_logger().warn("测试前请确保环境安全，最好先把机器人架空或留足空间。")

    def build_cmd_list(self, start_cmd, end_cmd, step_cmd):
        vals = []
        x = start_cmd
        while x <= end_cmd + 1e-9:
            vals.append(round(x, 6))
            x += step_cmd
        if vals[-1] != end_cmd:
            vals.append(end_cmd)
        return vals

    def odom_cb(self, msg: Odometry):
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        vx = float(msg.twist.twist.linear.x)
        vy = float(msg.twist.twist.linear.y)
        wz = float(msg.twist.twist.angular.z)

        if self.mode == 'x':
            fb = abs(vx)
        elif self.mode == 'y':
            fb = abs(vy)
        else:
            fb = abs(wz)

        self.feedback_buf.append((now_sec, fb))
        self.latest_odom_time = now_sec

        # 清掉窗口外旧数据
        while self.feedback_buf and now_sec - self.feedback_buf[0][0] > self.avg_window_sec:
            self.feedback_buf.popleft()

    def get_feedback_avg(self):
        if not self.feedback_buf:
            return None
        return sum(v for _, v in self.feedback_buf) / len(self.feedback_buf)

    def get_feedback_eps(self):
        if self.mode == 'x':
            return self.feedback_eps_x
        elif self.mode == 'y':
            return self.feedback_eps_y
        else:
            return self.feedback_eps_yaw

    def publish_cmd(self, x_vel, y_vel, yaw_vel):
        msg = NavCmd()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = 0

        msg.data.x_vel = float(x_vel)
        msg.data.y_vel = float(y_vel)
        msg.data.yaw_vel = float(yaw_vel)

        self.pub.publish(msg)

    def stop_robot(self):
        self.publish_cmd(0.0, 0.0, 0.0)

    def on_timer(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        if self.phase_idx >= len(self.cmd_values):
            self.stop_robot()
            if self.detected_threshold is not None:
                self.get_logger().warn(
                    f"测试结束：估计 {self.mode} 启动阈值约为 {self.detected_threshold:.3f}"
                )
            else:
                self.get_logger().warn(
                    f"测试结束：在 [{self.start_cmd}, {self.end_cmd}] 内没有检测到明显启动阈值"
                )
            rclpy.shutdown()
            return

        self.current_cmd = self.cmd_values[self.phase_idx]

        # 持续发布当前测试值
        if self.mode == 'x':
            self.publish_cmd(self.current_cmd, 0.0, 0.0)
        elif self.mode == 'y':
            self.publish_cmd(0.0, self.current_cmd, 0.0)
        else:
            self.publish_cmd(0.0, 0.0, self.current_cmd)

        elapsed = now_sec - self.phase_start_time
        if elapsed < self.hold_sec:
            return

        fb_avg = self.get_feedback_avg()
        eps = self.get_feedback_eps()

        if fb_avg is None:
            self.get_logger().warn(
                f"cmd={self.current_cmd:.3f}, 还没有收到 odom，无法判断。"
            )
        else:
            self.get_logger().warn(
                f"cmd={self.current_cmd:.3f}, avg_feedback={fb_avg:.4f}, move_eps={eps:.4f}"
            )

            if self.detected_threshold is None and fb_avg > eps:
                self.detected_threshold = self.current_cmd
                self.get_logger().warn(
                    f"检测到启动阈值：mode={self.mode}, threshold≈{self.detected_threshold:.3f}"
                )

        # 下一档
        self.phase_idx += 1
        self.phase_start_time = now_sec
        self.feedback_buf.clear()

    def destroy_node(self):
        try:
            self.stop_robot()
            time.sleep(0.2)
            self.stop_robot()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = DeadbandTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()


if __name__ == '__main__':
    main()