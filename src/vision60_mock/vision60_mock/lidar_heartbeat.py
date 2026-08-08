import math

from geometry_msgs.msg import TransformStamped
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import StaticTransformBroadcaster


LIDAR_TRANSLATION = (0.20, 0.0, 0.45)
CAMERA_TRANSLATION = (0.30, 0.0, 0.40)
CAMERA_OPTICAL_QUATERNION = (-0.5, 0.5, -0.5, 0.5)


def obstacle_phase(
    elapsed_s: float,
    slowdown_after_s: float,
    slowdown_duration_s: float,
    stop_after_s: float,
    stop_duration_s: float,
) -> str:
    if (
        stop_after_s >= 0.0
        and stop_after_s
        <= elapsed_s
        < stop_after_s + stop_duration_s
    ):
        return 'stop'
    if (
        slowdown_after_s >= 0.0
        and slowdown_after_s
        <= elapsed_s
        < slowdown_after_s + slowdown_duration_s
    ):
        return 'slowdown'
    return 'clear'


def points_for_phase(phase: str):
    if phase == 'slowdown':
        return [
            (0.90, -0.20, 0.30),
            (0.90, -0.10, 0.30),
            (0.90, 0.00, 0.30),
            (0.90, 0.10, 0.30),
            (0.90, 0.20, 0.30),
        ]
    if phase == 'stop':
        return [
            (0.40, -0.20, 0.30),
            (0.40, -0.10, 0.30),
            (0.40, 0.00, 0.30),
            (0.40, 0.10, 0.30),
            (0.40, 0.20, 0.30),
        ]
    return []


def static_scene_points():
    """Return a deterministic corridor in the mock odom frame."""
    values = []
    for y_index in range(31):
        y = -2.4 + y_index * 0.16
        for z_index in range(15):
            z = -0.5 + z_index * 0.14
            values.append((5.0, y, z))
    for side_y in (-2.4, 2.4):
        for x_index in range(25):
            x = 1.2 + x_index * 0.16
            for z_index in range(15):
                z = -0.5 + z_index * 0.14
                values.append((x, side_y, z))
    for x_index in range(25):
        x = 1.2 + x_index * 0.16
        for y_index in range(25):
            y = -2.4 + y_index * 0.20
            values.append((x, y, -0.5))
    return values


def world_to_base(points, x: float, y: float, yaw: float):
    """Transform fixed odom-frame points into the moving base frame."""
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    result = []
    for world_x, world_y, world_z in points:
        dx = world_x - x
        dy = world_y - y
        base_x = cosine * dx + sine * dy
        base_y = -sine * dx + cosine * dy
        if 0.25 <= base_x <= 7.0:
            result.append((base_x, base_y, world_z))
    return result


def base_to_lidar(points):
    """Express base-frame points in the aligned, translated LiDAR frame."""
    tx, ty, tz = LIDAR_TRANSLATION
    return [
        (x - tx, y - ty, z - tz)
        for x, y, z in points
    ]


def synthetic_camera_rgb(width: int, height: int) -> bytes:
    """Create a repeatable RGB image with gradients and texture."""
    pixels = bytearray(width * height * 3)
    index = 0
    for row in range(height):
        for column in range(width):
            checker = 55 if (row // 12 + column // 12) % 2 else 0
            pixels[index] = min(255, 35 + column + checker)
            pixels[index + 1] = min(255, 45 + row * 2)
            pixels[index + 2] = 210 - checker
            index += 3
    return bytes(pixels)


def quaternion_yaw(quaternion) -> float:
    """Return planar yaw from a geometry quaternion."""
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z
        + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def camera_info_message(stamp, frame_id: str, width: int, height: int):
    """Return calibrated pinhole parameters for the synchronized image."""
    fx = 110.0 * width / 160.0
    fy = 110.0 * height / 120.0
    cx = width / 2.0
    cy = height / 2.0
    message = CameraInfo()
    message.header.stamp = stamp
    message.header.frame_id = frame_id
    message.height = height
    message.width = width
    message.distortion_model = 'plumb_bob'
    message.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    message.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    message.r = [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    ]
    message.p = [
        fx, 0.0, cx, 0.0,
        0.0, fy, cy, 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    return message


def static_frame_transforms(stamp, lidar_frame: str, camera_frame: str):
    """Return the complete fixed frame tree used by the sensor mock."""
    map_to_odom = TransformStamped()
    map_to_odom.header.stamp = stamp
    map_to_odom.header.frame_id = 'map'
    map_to_odom.child_frame_id = 'odom'
    map_to_odom.transform.rotation.w = 1.0

    base_to_sensor = TransformStamped()
    base_to_sensor.header.stamp = stamp
    base_to_sensor.header.frame_id = 'base_link'
    base_to_sensor.child_frame_id = lidar_frame
    base_to_sensor.transform.translation.x = LIDAR_TRANSLATION[0]
    base_to_sensor.transform.translation.y = LIDAR_TRANSLATION[1]
    base_to_sensor.transform.translation.z = LIDAR_TRANSLATION[2]
    base_to_sensor.transform.rotation.w = 1.0

    base_to_camera = TransformStamped()
    base_to_camera.header.stamp = stamp
    base_to_camera.header.frame_id = 'base_link'
    base_to_camera.child_frame_id = camera_frame
    base_to_camera.transform.translation.x = CAMERA_TRANSLATION[0]
    base_to_camera.transform.translation.y = CAMERA_TRANSLATION[1]
    base_to_camera.transform.translation.z = CAMERA_TRANSLATION[2]
    quaternion = CAMERA_OPTICAL_QUATERNION
    base_to_camera.transform.rotation.x = quaternion[0]
    base_to_camera.transform.rotation.y = quaternion[1]
    base_to_camera.transform.rotation.z = quaternion[2]
    base_to_camera.transform.rotation.w = quaternion[3]
    return [map_to_odom, base_to_sensor, base_to_camera]


class MockLidarHeartbeat(Node):
    """
    Publish synchronized scene LiDAR and camera data for safety and mapping.

    The real system feeds this topic from the Ouster driver. The fixed
    corridor is transformed by mock odometry, while foreground
    obstacle phases still exercise collision slowdown and stop behavior.
    Setting ``silent`` to true exercises the bridge LiDAR fail-safe.
    """

    def __init__(self) -> None:
        super().__init__('mock_lidar_heartbeat')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('frame_id', 'os_sensor')
        self.declare_parameter('output_topic', '/ouster/points')
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_frame_id', 'camera_optical')
        self.declare_parameter('camera_width', 160)
        self.declare_parameter('camera_height', 120)
        self.declare_parameter('camera_publish_divisor', 2)
        self.declare_parameter('silent', False)
        self.declare_parameter('slowdown_after_s', -1.0)
        self.declare_parameter('slowdown_duration_s', 0.0)
        self.declare_parameter('stop_after_s', -1.0)
        self.declare_parameter('stop_duration_s', 0.0)

        self._frame_id = str(self.get_parameter('frame_id').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self._camera_frame_id = str(
            self.get_parameter('camera_frame_id').value
        )
        self._camera_width = int(self.get_parameter('camera_width').value)
        self._camera_height = int(
            self.get_parameter('camera_height').value
        )
        self._camera_publish_divisor = int(
            self.get_parameter('camera_publish_divisor').value
        )
        if self._camera_publish_divisor <= 0:
            raise ValueError('camera_publish_divisor must be positive')
        self._camera_data = synthetic_camera_rgb(
            self._camera_width, self._camera_height
        )
        self._world_scene = static_scene_points()
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._tick_count = 0
        self._start_ns = self.get_clock().now().nanoseconds
        self._publisher = self.create_publisher(
            PointCloud2, output_topic, 10
        )
        self._phase_publisher = self.create_publisher(
            String, '/vision60/mock_obstacle_phase', 10
        )
        self._camera_publisher = self.create_publisher(
            Image,
            str(self.get_parameter('camera_topic').value),
            10,
        )
        self._camera_info_publisher = self.create_publisher(
            CameraInfo, '/camera/camera_info', 10
        )
        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._static_broadcaster.sendTransform(static_frame_transforms(
            self.get_clock().now().to_msg(),
            self._frame_id,
            self._camera_frame_id,
        ))
        self.create_subscription(
            Odometry, '/vision60/odom', self._odometry_callback, 20
        )
        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / rate_hz, self._tick)

    def _odometry_callback(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self._robot_x = float(position.x)
        self._robot_y = float(position.y)
        self._robot_yaw = quaternion_yaw(
            message.pose.pose.orientation
        )

    def _tick(self) -> None:
        if bool(self.get_parameter('silent').value):
            return
        now = self.get_clock().now()
        elapsed_s = (now.nanoseconds - self._start_ns) / 1e9
        phase = obstacle_phase(
            elapsed_s,
            float(self.get_parameter('slowdown_after_s').value),
            float(self.get_parameter('slowdown_duration_s').value),
            float(self.get_parameter('stop_after_s').value),
            float(self.get_parameter('stop_duration_s').value),
        )
        header = Header()
        header.stamp = now.to_msg()
        header.frame_id = self._frame_id
        scene_points = world_to_base(
            self._world_scene,
            self._robot_x,
            self._robot_y,
            self._robot_yaw,
        )
        scene_points.extend(points_for_phase(phase))
        lidar_points = base_to_lidar(scene_points)
        message = point_cloud2.create_cloud_xyz32(
            header,
            lidar_points,
        )
        self._publisher.publish(message)
        phase_message = String()
        phase_message.data = phase
        self._phase_publisher.publish(phase_message)
        if self._tick_count % self._camera_publish_divisor == 0:
            image = Image()
            image.header.stamp = header.stamp
            image.header.frame_id = self._camera_frame_id
            image.height = self._camera_height
            image.width = self._camera_width
            image.encoding = 'rgb8'
            image.is_bigendian = False
            image.step = self._camera_width * 3
            image.data = self._camera_data
            self._camera_publisher.publish(image)
            info = camera_info_message(
                header.stamp,
                self._camera_frame_id,
                self._camera_width,
                self._camera_height,
            )
            self._camera_info_publisher.publish(info)
        self._tick_count += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockLidarHeartbeat()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
