import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from sensor_msgs.msg import Imu

from tf2_ros import TransformBroadcaster

from drdds.msg import MotionInfo


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class SimpleOdomNode(Node):
    def __init__(self):
        super().__init__('simple_odom_node')

        self.declare_parameter('motion_topic', '/MOTION_INFO')
        self.declare_parameter('imu_topic', '/IMU')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('use_imu_yaw', True)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('publish_tf', True)

        motion_topic = self.get_parameter('motion_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.use_imu_yaw = self.get_parameter('use_imu_yaw').value
        self.publish_tf = self.get_parameter('publish_tf').value
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.motion_sub = self.create_subscription(
            MotionInfo,
            motion_topic,
            self.motion_callback,
            20
        )

        self.imu_sub = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            50
        )

        self.odom_pub = self.create_publisher(Odometry, odom_topic, 20)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        # 状态量
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0

        self.vel_x_body = 0.0
        self.vel_y_body = 0.0
        self.vel_yaw = 0.0

        self.imu_yaw = 0.0
        self.has_imu = False
        self.last_time = self.get_clock().now()

        self.get_logger().info('simple_odom_node started.')

    def motion_callback(self, msg: MotionInfo):
        self.vel_x_body = float(msg.data.vel_x)
        self.vel_y_body = float(msg.data.vel_y)
        self.vel_yaw = float(msg.data.vel_yaw)
        self.z = float(msg.data.height)

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if norm > 1e-6:
            self.imu_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            self.has_imu = True

    def publish_odom_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = self.z

        q = yaw_to_quaternion(self.yaw)
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w

        self.tf_broadcaster.sendTransform(t)

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 0.5:
            self.last_time = now
            return

        # yaw 来源：
        # 1) 优先用 IMU 的姿态
        # 2) 没有 IMU 姿态就积分 vel_yaw
        if self.use_imu_yaw and self.has_imu:
            self.yaw = self.imu_yaw
        else:
            self.yaw += self.vel_yaw * dt

        # body frame -> odom frame
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        vx_odom = cos_yaw * self.vel_x_body - sin_yaw * self.vel_y_body
        vy_odom = sin_yaw * self.vel_x_body + cos_yaw * self.vel_y_body

        # 积分位置
        self.x += vx_odom * dt
        self.y += vy_odom * dt

        q = yaw_to_quaternion(self.yaw)

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # pose
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = self.z
        odom_msg.pose.pose.orientation = q

        # twist 默认表示在 child_frame_id(base_link) 下
        odom_msg.twist.twist.linear.x = self.vel_x_body
        odom_msg.twist.twist.linear.y = self.vel_y_body
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.z = self.vel_yaw

        # 简单协方差
        pose_cov = [0.0] * 36
        twist_cov = [0.0] * 36

        pose_cov[0] = 0.05
        pose_cov[7] = 0.05
        pose_cov[14] = 0.1
        pose_cov[21] = 0.2
        pose_cov[28] = 0.2
        pose_cov[35] = 0.1

        twist_cov[0] = 0.05
        twist_cov[7] = 0.05
        twist_cov[14] = 0.1
        twist_cov[21] = 0.2
        twist_cov[28] = 0.2
        twist_cov[35] = 0.1

        odom_msg.pose.covariance = pose_cov
        odom_msg.twist.covariance = twist_cov

        self.odom_pub.publish(odom_msg)

        if self.publish_tf:
            self.publish_odom_tf(now.to_msg())

        self.last_time = now


def main(args=None):
    rclpy.init(args=args)
    node = SimpleOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()