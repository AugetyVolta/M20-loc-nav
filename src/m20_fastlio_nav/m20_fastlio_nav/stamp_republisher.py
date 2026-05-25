#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Shift Livox sensor message stamps onto the active ROS clock.

The MID360 driver can publish hardware-time stamps that are several days away
from the host wall clock when the sensor clock is not synchronized.  Point-LIO,
Nav2, RViz, and pointcloud_to_laserscan all require message stamps and dynamic
TF stamps to live on the same timeline.

This node keeps the payload and frame_id unchanged, computes one offset from the
first non-zero sensor stamp to ``node.get_clock().now()``, and applies that
constant offset to subsequent cloud and IMU messages.  That preserves the
relative LiDAR/IMU timing while moving the whole stream onto the ROS clock used
by TF.
"""

import math

from builtin_interfaces.msg import Time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2


class StampRepublisher(Node):
    def __init__(self) -> None:
        super().__init__("stamp_republisher")

        self.declare_parameter("cloud_in_topic", "/livox/lidar")
        self.declare_parameter("cloud_out_topic", "/livox/lidar_stamped")
        self.declare_parameter("imu_in_topic", "/livox/imu")
        self.declare_parameter("imu_out_topic", "/livox/imu_stamped")

        self.cloud_in_topic = str(self.get_parameter("cloud_in_topic").value)
        self.cloud_out_topic = str(self.get_parameter("cloud_out_topic").value)
        self.imu_in_topic = str(self.get_parameter("imu_in_topic").value)
        self.imu_out_topic = str(self.get_parameter("imu_out_topic").value)

        self.cloud_pub = self.create_publisher(
            PointCloud2, self.cloud_out_topic, qos_profile_sensor_data
        )
        self.imu_pub = self.create_publisher(Imu, self.imu_out_topic, qos_profile_sensor_data)

        self.cloud_sub = self.create_subscription(
            PointCloud2, self.cloud_in_topic, self._on_cloud, qos_profile_sensor_data
        )
        self.imu_sub = self.create_subscription(
            Imu, self.imu_in_topic, self._on_imu, qos_profile_sensor_data
        )

        self._stamp_offset_sec = None

        self.get_logger().info(
            "shifting sensor stamps: "
            f"{self.cloud_in_topic} -> {self.cloud_out_topic}, "
            f"{self.imu_in_topic} -> {self.imu_out_topic}"
        )

    @staticmethod
    def _stamp_to_sec(stamp: Time) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    @staticmethod
    def _sec_to_stamp(value: float) -> Time:
        if not math.isfinite(value) or value < 0.0:
            value = 0.0
        sec = int(math.floor(value))
        nanosec = int(round((value - sec) * 1.0e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        return Time(sec=sec, nanosec=nanosec)

    def _shift_stamp(self, stamp: Time) -> Time:
        original_sec = self._stamp_to_sec(stamp)
        now_sec = self.get_clock().now().nanoseconds * 1.0e-9
        if original_sec <= 0.0 or not math.isfinite(original_sec):
            return self._sec_to_stamp(now_sec)

        if self._stamp_offset_sec is None:
            self._stamp_offset_sec = now_sec - original_sec
            self.get_logger().info(
                "sensor stamp offset initialized: "
                f"ros_now={now_sec:.6f}, sensor={original_sec:.6f}, "
                f"offset={self._stamp_offset_sec:.6f}s"
            )

        return self._sec_to_stamp(original_sec + self._stamp_offset_sec)

    def _on_cloud(self, msg: PointCloud2) -> None:
        msg.header.stamp = self._shift_stamp(msg.header.stamp)
        self.cloud_pub.publish(msg)

    def _on_imu(self, msg: Imu) -> None:
        msg.header.stamp = self._shift_stamp(msg.header.stamp)
        self.imu_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StampRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
