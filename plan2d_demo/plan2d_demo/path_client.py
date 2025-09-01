#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose

class PlanClient(Node):
    def __init__(self):
        super().__init__('plan_client')
        self.start_pose: PoseStamped | None = None

        self.start_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', self._start_cb, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self._goal_cb, 10)

        self.path_pub = self.create_publisher(Path, 'planned_path', 10)
        self.client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')

        self.get_logger().info('PlanClient ready. Set start with "2D Pose Estimate" then goal with "2D Nav Goal".')
        self.follow_client = ActionClient(self, FollowPath, 'follow_path')  # ★

    def _start_cb(self, msg: PoseWithCovarianceStamped):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.start_pose = ps
        self.get_logger().info('Start pose set.')

    def _goal_cb(self, goal: PoseStamped):
        if self.start_pose is None:
            self.get_logger().warn('Set start pose first (RViz: 2D Pose Estimate).')
            return

        if not self.client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Planner action server not available.')
            return

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = self.start_pose
        goal_msg.goal = goal
        goal_msg.use_start = False            # 시작자세를 명시적으로 사용
        goal_msg.planner_id = 'GridBased'    # nav2_params.yaml의 플러그인 이름과 일치

        self.get_logger().info('Requesting path...')
        send_future = self.client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, fut):
        goal_handle = fut.result()
        if not goal_handle.accepted:
            self.get_logger().error('Path request rejected.')
            return
        self.get_logger().info('Path request accepted; waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, fut):
        result = fut.result().result
        path: Path = result.path
        self.path_pub.publish(path)
        self.get_logger().info(f'Got path with {len(path.poses)} poses.')

        # 바로 주행 시작 (DWB 사용)
        if not self.follow_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('FollowPath server not available.')
            return

        fp_goal = FollowPath.Goal()
        fp_goal.path = path
        fp_goal.controller_id = 'FollowPath'   # controller_plugins의 키와 동일
        fp_goal.goal_checker_id = 'goal_checker'

        self.get_logger().info('Sending FollowPath goal...')
        f = self.follow_client.send_goal_async(fp_goal)
        f.add_done_callback(self._on_follow_sent)

    def _on_follow_sent(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error('FollowPath goal rejected.')
            return
        self.get_logger().info('FollowPath accepted; waiting result...')
        rf = gh.get_result_async()
        rf.add_done_callback(lambda r: self.get_logger().info('FollowPath finished.'))

def main():
    rclpy.init()
    node = PlanClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
