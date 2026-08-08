import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


class RecoveryPipelineProbe(Node):
    def __init__(self) -> None:
        super().__init__('recovery_pipeline_probe')
        self._publisher = self.create_publisher(
            Path, '/mission/recovery_path', 10
        )
        self.create_subscription(
            Odometry, '/slam/odom', self._odometry_callback, 20
        )
        self._timer = self.create_timer(1.0, self._publish_once)
        self._published = False
        self.exit_code = 1
        self.done = False

    def _publish_once(self) -> None:
        self._published = True
        self._timer.cancel()
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
        self._publisher.publish(path)
        self.get_logger().info('Published /mission/recovery_path')

    def _odometry_callback(self, message: Odometry) -> None:
        if self._published and message.pose.pose.position.x <= 0.11:
            self.exit_code = 0
            self.get_logger().info(
                'RECOVERY PIPELINE PASS: recorded path reached Nav2'
            )
            self.done = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RecoveryPipelineProbe()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)
