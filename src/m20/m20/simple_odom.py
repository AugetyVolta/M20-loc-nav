import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from sensor_msgs.msg import Imu

from tf2_ros import TransformBroadcaster, Buffer, TransformListener, TransformException

from drdds.msg import MotionInfo


def normalize_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def apply_deadband(v: float, deadband: float) -> float:
    if abs(v) < deadband:
        return 0.0
    return v


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


def quat_normalize(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return 0.0, 0.0, 0.0, 1.0
    return x / n, y / n, z / n, w / n


def quat_inverse(q):
    x, y, z, w = quat_normalize(q)
    return -x, -y, -z, w


def quat_multiply_raw(q1, q2):
    """
    四元数乘法，不做归一化。
    用于旋转向量时不能中间归一化，否则会破坏角速度向量的模长。
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2

    return x, y, z, w


def quat_multiply(q1, q2):
    return quat_normalize(quat_multiply_raw(q1, q2))


def rotate_vector_by_quat(q, v):
    """
    用四元数 q 旋转向量 v。

    q 表示 source_frame -> target_frame 的旋转时，
    返回的就是 v 在 target_frame 下的表达。
    """
    q = quat_normalize(q)
    q_inv = quat_inverse(q)

    vx, vy, vz = v
    v_quat = vx, vy, vz, 0.0

    tmp = quat_multiply_raw(q, v_quat)
    rotated = quat_multiply_raw(tmp, q_inv)

    return rotated[0], rotated[1], rotated[2]


class SimpleOdomNode(Node):
    def __init__(self):
        super().__init__('simple_odom_node')

        # -----------------------------
        # 参数
        # -----------------------------
        self.declare_parameter('motion_topic', '/MOTION_INFO')
        self.declare_parameter('imu_topic', '/livox/imu')
        self.declare_parameter('odom_topic', '/odom')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('imu_frame', 'livox_frame')

        # MID360 IMU 设置
        self.declare_parameter('use_imu_angular_velocity', True)
        self.declare_parameter('use_imu_orientation', False)
        self.declare_parameter('transform_imu_to_base', True)

        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('imu_timeout', 0.2)

        # MID360 gyro 零偏标定
        self.declare_parameter('gyro_bias_calib_time', 3.0)
        self.declare_parameter('gyro_deadband', 0.015)
        self.declare_parameter('gyro_z_sign', 1.0)

        # /MOTION_INFO 速度死区
        # 你静止时 vel_x≈0.005, vel_y≈-0.0037，所以 0.015 可以压掉静止漂移
        self.declare_parameter('linear_deadband', 0.015)
        self.declare_parameter('angular_deadband', 0.01)

        # 静止锁定阈值：判定静止时不积分 x/y
        self.declare_parameter('stationary_linear_threshold', 0.02)
        self.declare_parameter('stationary_angular_threshold', 0.015)

        # TF warning 节流
        self.declare_parameter('tf_warn_interval', 2.0)

        motion_topic = self.get_parameter('motion_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        odom_topic = self.get_parameter('odom_topic').value

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.imu_frame = self.get_parameter('imu_frame').value

        self.use_imu_angular_velocity = bool(
            self.get_parameter('use_imu_angular_velocity').value
        )
        self.use_imu_orientation = bool(
            self.get_parameter('use_imu_orientation').value
        )
        self.transform_imu_to_base = bool(
            self.get_parameter('transform_imu_to_base').value
        )

        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.imu_timeout = float(self.get_parameter('imu_timeout').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.gyro_bias_calib_time = float(
            self.get_parameter('gyro_bias_calib_time').value
        )
        self.gyro_deadband = float(
            self.get_parameter('gyro_deadband').value
        )
        self.gyro_z_sign = float(
            self.get_parameter('gyro_z_sign').value
        )

        self.linear_deadband = float(
            self.get_parameter('linear_deadband').value
        )
        self.angular_deadband = float(
            self.get_parameter('angular_deadband').value
        )
        self.stationary_linear_threshold = float(
            self.get_parameter('stationary_linear_threshold').value
        )
        self.stationary_angular_threshold = float(
            self.get_parameter('stationary_angular_threshold').value
        )

        self.tf_warn_interval = float(
            self.get_parameter('tf_warn_interval').value
        )

        # -----------------------------
        # TF
        # -----------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # -----------------------------
        # 订阅和发布
        # -----------------------------
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
            100
        )

        self.odom_pub = self.create_publisher(Odometry, odom_topic, 20)

        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)

        # -----------------------------
        # odom 状态量
        # -----------------------------
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0

        # 来自 /MOTION_INFO 的机体系速度
        self.vel_x_body = 0.0
        self.vel_y_body = 0.0
        self.vel_yaw = 0.0

        # 原始速度，仅用于调试/判断
        self.raw_vel_x_body = 0.0
        self.raw_vel_y_body = 0.0
        self.raw_vel_yaw = 0.0

        # MID360 IMU 转换到 base_link 后的 yaw rate
        self.imu_omega_z_base = 0.0

        # 可选：如果以后 MID360 orientation 变可靠，可以打开
        self.imu_yaw = 0.0
        self.imu_yaw0 = None
        self.has_imu_orientation = False

        # IMU 状态
        self.has_imu = False
        self.last_imu_time = None

        # gyro 零偏标定状态
        self.gyro_bias_z = 0.0
        self.gyro_bias_sum = 0.0
        self.gyro_bias_count = 0
        self.gyro_bias_start_time = None
        self.gyro_bias_ready = False

        self.last_time = self.get_clock().now()
        self.last_tf_warn_time = None

        self.get_logger().info(
            'simple_odom_node started. '
            f'motion_topic={motion_topic}, '
            f'imu_topic={imu_topic}, '
            f'odom_topic={odom_topic}, '
            f'imu_frame={self.imu_frame}, '
            f'base_frame={self.base_frame}'
        )

        self.get_logger().info(
            'Using MID360 angular_velocity for yaw. '
            f'gyro_bias_calib_time={self.gyro_bias_calib_time}s, '
            f'gyro_deadband={self.gyro_deadband} rad/s, '
            f'gyro_z_sign={self.gyro_z_sign}'
        )

        self.get_logger().info(
            'MotionInfo deadband enabled. '
            f'linear_deadband={self.linear_deadband} m/s, '
            f'angular_deadband={self.angular_deadband} rad/s, '
            f'stationary_linear_threshold={self.stationary_linear_threshold} m/s, '
            f'stationary_angular_threshold={self.stationary_angular_threshold} rad/s'
        )

    def motion_callback(self, msg: MotionInfo):
        raw_vx = float(msg.data.vel_x)
        raw_vy = float(msg.data.vel_y)
        raw_wz = float(msg.data.vel_yaw)

        self.raw_vel_x_body = raw_vx
        self.raw_vel_y_body = raw_vy
        self.raw_vel_yaw = raw_wz

        # 压掉机器狗静止时 /MOTION_INFO 的小速度偏置
        self.vel_x_body = apply_deadband(raw_vx, self.linear_deadband)
        self.vel_y_body = apply_deadband(raw_vy, self.linear_deadband)
        self.vel_yaw = apply_deadband(raw_wz, self.angular_deadband)

        self.z = float(msg.data.height)

    def warn_tf_throttled(self, text: str):
        now = self.get_clock().now()

        if self.last_tf_warn_time is None:
            self.last_tf_warn_time = now
            self.get_logger().warn(text)
            return

        dt = (now - self.last_tf_warn_time).nanoseconds * 1e-9

        if dt >= self.tf_warn_interval:
            self.last_tf_warn_time = now
            self.get_logger().warn(text)

    def get_imu_to_base_rotation(self, imu_frame_from_msg: str):
        """
        返回 q_base_imu:
            imu_frame -> base_link 的旋转四元数。

        lookup_transform(target, source, time)
        这里 target 是 base_link，source 是 livox_frame。
        """
        imu_frame = imu_frame_from_msg if imu_frame_from_msg else self.imu_frame

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.base_frame,
                imu_frame,
                Time()
            )

            q = tf_msg.transform.rotation
            return quat_normalize((q.x, q.y, q.z, q.w))

        except TransformException as ex:
            self.warn_tf_throttled(
                f'Cannot transform {imu_frame} to {self.base_frame}: {ex}'
            )
            return None

    def imu_callback(self, msg: Imu):
        """
        MID360 IMU:
        - orientation 当前通常是 0,0,0,1，不使用
        - angular_velocity 可用，但必须：
            1. 从 livox_frame 转到 base_link
            2. 做 z 轴零偏标定
            3. 加死区
        """
        imu_frame = msg.header.frame_id if msg.header.frame_id else self.imu_frame

        omega_imu = (
            float(msg.angular_velocity.x),
            float(msg.angular_velocity.y),
            float(msg.angular_velocity.z),
        )

        # 1. angular_velocity 从 livox_frame 转到 base_link
        if self.transform_imu_to_base:
            q_base_imu = self.get_imu_to_base_rotation(imu_frame)
            if q_base_imu is None:
                return

            omega_base = rotate_vector_by_quat(q_base_imu, omega_imu)
        else:
            omega_base = omega_imu

        raw_omega_z_base = float(omega_base[2])

        now = self.get_clock().now()

        # 2. 启动后静止标定 gyro z 零偏
        if not self.gyro_bias_ready:
            if self.gyro_bias_start_time is None:
                self.gyro_bias_start_time = now
                self.get_logger().info(
                    'Start calibrating MID360 gyro bias. Keep robot still.'
                )

            elapsed = (now - self.gyro_bias_start_time).nanoseconds * 1e-9

            self.gyro_bias_sum += raw_omega_z_base
            self.gyro_bias_count += 1

            if elapsed >= self.gyro_bias_calib_time and self.gyro_bias_count > 0:
                self.gyro_bias_z = self.gyro_bias_sum / self.gyro_bias_count
                self.gyro_bias_ready = True
                self.get_logger().info(
                    f'MID360 gyro bias calibrated: '
                    f'bias_z={self.gyro_bias_z:.6f} rad/s, '
                    f'samples={self.gyro_bias_count}'
                )

            # 标定期间不让 odom yaw 乱转
            self.imu_omega_z_base = 0.0

        else:
            omega_z = raw_omega_z_base - self.gyro_bias_z

            # 3. 死区：静止小抖动直接置零
            if abs(omega_z) < self.gyro_deadband:
                omega_z = 0.0

            self.imu_omega_z_base = self.gyro_z_sign * omega_z

        # 4. 可选：如果以后 IMU orientation 可用，可以打开 use_imu_orientation
        if self.use_imu_orientation:
            q_msg = msg.orientation
            norm = (
                q_msg.x * q_msg.x
                + q_msg.y * q_msg.y
                + q_msg.z * q_msg.z
                + q_msg.w * q_msg.w
            )

            orientation_valid = norm > 1e-6

            if len(msg.orientation_covariance) > 0:
                if msg.orientation_covariance[0] == -1.0:
                    orientation_valid = False

            if orientation_valid:
                q_odom_imu = quat_normalize(
                    (q_msg.x, q_msg.y, q_msg.z, q_msg.w)
                )

                if self.transform_imu_to_base:
                    q_base_imu = self.get_imu_to_base_rotation(imu_frame)
                    if q_base_imu is None:
                        return

                    # q_odom_imu = q_odom_base * q_base_imu
                    # q_odom_base = q_odom_imu * inverse(q_base_imu)
                    q_odom_base = quat_multiply(
                        q_odom_imu,
                        quat_inverse(q_base_imu)
                    )
                else:
                    q_odom_base = q_odom_imu

                yaw_abs = quaternion_to_yaw(*q_odom_base)

                if self.imu_yaw0 is None:
                    self.imu_yaw0 = yaw_abs
                    self.get_logger().info(
                        f'Initial MID360 IMU yaw set to {yaw_abs:.3f} rad'
                    )

                self.imu_yaw = normalize_angle(yaw_abs - self.imu_yaw0)
                self.has_imu_orientation = True

        self.has_imu = True
        self.last_imu_time = now

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

        imu_ok = False
        if self.has_imu and self.last_imu_time is not None:
            imu_age = (now - self.last_imu_time).nanoseconds * 1e-9
            imu_ok = imu_age < self.imu_timeout

        # -----------------------------
        # yaw 更新
        # -----------------------------
        if self.use_imu_orientation and self.has_imu_orientation and imu_ok:
            # 当前不建议使用，因为 MID360 orientation 基本是 0,0,0,1
            self.yaw = self.imu_yaw

        elif self.use_imu_angular_velocity and imu_ok:
            # 使用 MID360 gyro，已经经过：
            # livox_frame -> base_link
            # bias calibration
            # deadband
            self.yaw += self.imu_omega_z_base * dt
            self.yaw = normalize_angle(self.yaw)

        else:
            # 如果 MID360 IMU 不可用，退回 /MOTION_INFO 里的 vel_yaw
            self.yaw += self.vel_yaw * dt
            self.yaw = normalize_angle(self.yaw)

        # -----------------------------
        # body frame -> odom frame
        # -----------------------------
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        vx_odom = cos_yaw * self.vel_x_body - sin_yaw * self.vel_y_body
        vy_odom = sin_yaw * self.vel_x_body + cos_yaw * self.vel_y_body

        # -----------------------------
        # 静止锁定：狗不动时不积分 x/y
        # -----------------------------
        linear_speed = math.hypot(self.vel_x_body, self.vel_y_body)

        if self.use_imu_angular_velocity and imu_ok:
            yaw_rate_for_stationary_check = self.imu_omega_z_base
        else:
            yaw_rate_for_stationary_check = self.vel_yaw

        is_stationary = (
            linear_speed < self.stationary_linear_threshold and
            abs(yaw_rate_for_stationary_check) < self.stationary_angular_threshold
        )

        if not is_stationary:
            self.x += vx_odom * dt
            self.y += vy_odom * dt

        # -----------------------------
        # 发布 Odometry
        # -----------------------------
        q = yaw_to_quaternion(self.yaw)

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = self.z
        odom_msg.pose.pose.orientation = q

        # twist 按 ROS 习惯表示在 child_frame_id，也就是 base_link 下
        odom_msg.twist.twist.linear.x = self.vel_x_body
        odom_msg.twist.twist.linear.y = self.vel_y_body
        odom_msg.twist.twist.linear.z = 0.0

        if self.use_imu_angular_velocity and imu_ok:
            odom_msg.twist.twist.angular.z = self.imu_omega_z_base
        else:
            odom_msg.twist.twist.angular.z = self.vel_yaw

        # 协方差：简单给一个可用值
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