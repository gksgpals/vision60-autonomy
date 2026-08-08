import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node


class FollowPathProbe(Node):
    def __init__(self) -> None:
        super().__init__('follow_path_probe')
        self._client = ActionClient(self, FollowPath, '/follow_path')
        self._timer = self.create_timer(0.5, self._send_once)
        self._sent = False
        self.exit_code = 1

    def _send_once(self) -> None:
        if self._sent or not self._client.wait_for_server(timeout_sec=0.1):
            return
        self._sent = True
        self._timer.cancel()
        goal = FollowPath.Goal()
        goal.path = self._make_return_path()
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)
        self.get_logger().info('Sent reverse path: x=2.0 -> 0.0')

    def _make_return_path(self) -> Path:
        path = Path()
        path.header.frame_id = 'odom'
        path.header.stamp = self.get_clock().now().to_msg()
        for index in range(21):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = 2.0 - index * 0.1
            pose.pose.orientation.z = math.sin(math.pi / 2.0)
            pose.pose.orientation.w = math.cos(math.pi / 2.0)
            path.poses.append(pose)
        return path

    def _goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('FollowPath goal rejected')
            rclpy.shutdown()
            return
        handle.get_result_async().add_done_callback(self._result)

    def _result(self, future) -> None:
        status = future.result().status
        self.exit_code = 0 if status == 4 else 1
        message = (
            'FOLLOW_PATH PASS: recovery path completed'
            if self.exit_code == 0
            else f'FOLLOW_PATH FAIL: status={status}'
        )
        (self.get_logger().info if self.exit_code == 0
         else self.get_logger().error)(message)
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowPathProbe()
    try:
        rclpy.spin(node)
    finally:
        exit_code = node.exit_code
        node.destroy_node()
    sys.exit(exit_code)
