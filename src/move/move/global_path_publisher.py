import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient

import numpy as np

class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner_node')
        self.get_logger().debug("Global Planner Node Started")

        self.declare_parameter('plan_period', 2.0)
        self.plan_period = max(0.5, float(self.get_parameter('plan_period').value))
        self.request_in_flight = False

        # 创建动作客户端
        self.compute_path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.publisher_ = self.create_publisher(Path, 'global_path', 20)

        # 设置目标点（可以改成参数或订阅接口）
        # -2.02258, -29.2049
        # -3.0,-19
        self.goal_xy = np.array([3.73781, -44.5877])  # [x, y] 目标位置
        self.timer = self.create_timer(self.plan_period, self.plan_and_publish_path)

    def plan_and_publish_path(self):
        if self.request_in_flight:
            return
        if not self.compute_path_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().debug("ComputePathToPose server not available yet.")
            return

        goal_msg = ComputePathToPose.Goal()
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map'
        target_pose.pose.position.x = float(self.goal_xy[0])
        target_pose.pose.position.y = float(self.goal_xy[1])
        target_pose.pose.orientation.w = 1.0

        goal_msg.goal = target_pose
        goal_msg.use_start = False  # 使用导航栈提供的当前位置

        self.get_logger().debug("Sending global path planning request...")

        self.request_in_flight = True
        send_goal_future = self.compute_path_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.request_in_flight = False
            self.get_logger().error(f'Goal request failed: {exc}')
            return
        if not goal_handle.accepted:
            self.request_in_flight = False
            self.get_logger().error('Goal rejected by server')
            return

        self.get_logger().debug('Goal accepted, waiting for result...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.request_in_flight = False
        result = future.result().result
        if result is None:
            self.get_logger().error('No path returned')
            return

        path_msg = result.path
        path_msg.header.frame_id = "map"
        self.get_logger().debug("publish global path")
        self.publisher_.publish(path_msg)
        self.get_logger().debug(f"Published global path with {len(path_msg.poses)} poses")

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
