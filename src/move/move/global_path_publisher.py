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
        self.get_logger().info("Global Planner Node Started")

        # 创建动作客户端
        self.compute_path_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.publisher_ = self.create_publisher(Path, 'global_path', 20)

        # 设置目标点（可以改成参数或订阅接口）
        # -2.02258, -29.2049
        # -9.90299, -0.0610579
        # 4.30133, 0.576853
        # 3.73781, -44.5877
        self.goal_xy = np.array([4.30133, 0.576853])  # [x, y] 目标位置
        self.timer = self.create_timer(1.0, self.plan_and_publish_path)

    def plan_and_publish_path(self):
        if not self.compute_path_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("ComputePathToPose server not available yet.")
            return

        goal_msg = ComputePathToPose.Goal()
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map'
        target_pose.pose.position.x = float(self.goal_xy[0])
        target_pose.pose.position.y = float(self.goal_xy[1])
        target_pose.pose.orientation.w = 1.0

        goal_msg.goal = target_pose
        goal_msg.use_start = False  # 使用导航栈提供的当前位置

        self.get_logger().info("Sending global path planning request...")

        send_goal_future = self.compute_path_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result is None:
            self.get_logger().error('No path returned')
            return

        path_msg = result.path
        path_msg.header.frame_id = "map"
        self.get_logger().warn(f"publish global path")
        self.publisher_.publish(path_msg)
        self.get_logger().info(f"Published global path with {len(path_msg.poses)} poses")

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
