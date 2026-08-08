"""Pure image and LiDAR fusion helpers used by mission perception."""

from dataclasses import dataclass
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class ColorTarget:
    """Simulation-only target definition."""

    class_id: str
    lower_hsv: tuple
    upper_hsv: tuple
    confidence: float


@dataclass(frozen=True)
class ImageDetection:
    """A class label and pixel bounding box."""

    class_id: str
    confidence: float
    x_min: int
    y_min: int
    x_max: int
    y_max: int

    @property
    def center(self):
        """Return the bounding-box center."""
        return (
            0.5 * (self.x_min + self.x_max),
            0.5 * (self.y_min + self.y_max),
        )


def normalize_external_detection(
    class_id,
    confidence,
    center_x,
    center_y,
    width,
    height,
    image_width,
    image_height,
    class_map,
    minimum_confidence=0.35,
):
    """Convert a detector box into a clipped internal candidate detection."""
    mapped_class = class_map.get(str(class_id))
    if mapped_class is None or confidence < minimum_confidence:
        return None
    values = np.asarray(
        [confidence, center_x, center_y, width, height], dtype=np.float64
    )
    if not np.isfinite(values).all() or width <= 0.0 or height <= 0.0:
        return None
    x_min = max(0, int(math.floor(center_x - width * 0.5)))
    y_min = max(0, int(math.floor(center_y - height * 0.5)))
    x_max = min(int(image_width) - 1, int(math.ceil(center_x + width * 0.5)))
    y_max = min(int(image_height) - 1, int(math.ceil(center_y + height * 0.5)))
    if x_max <= x_min or y_max <= y_min:
        return None
    return ImageDetection(
        mapped_class,
        float(confidence),
        x_min,
        y_min,
        x_max,
        y_max,
    )


SIMULATION_TARGETS = (
    ColorTarget('victim_candidate', (135, 90, 70), (179, 255, 255), 0.94),
    ColorTarget('hazard_candidate', (18, 90, 70), (42, 255, 255), 0.92),
)


def detect_simulation_targets(image_bgr, minimum_area=18):
    """Detect color-coded digital-twin targets, never real-world objects."""
    if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ValueError('image_bgr must have shape (height, width, 3)')
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    detections = []
    for target in SIMULATION_TARGETS:
        mask = cv2.inRange(
            hsv,
            np.asarray(target.lower_hsv, dtype=np.uint8),
            np.asarray(target.upper_hsv, dtype=np.uint8),
        )
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE, np.ones((3, 3), dtype=np.uint8)
        )
        count, _, stats, _ = cv2.connectedComponentsWithStats(mask)
        if count <= 1:
            continue
        component = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        area = int(stats[component, cv2.CC_STAT_AREA])
        if area < minimum_area:
            continue
        x_value = int(stats[component, cv2.CC_STAT_LEFT])
        y_value = int(stats[component, cv2.CC_STAT_TOP])
        width = int(stats[component, cv2.CC_STAT_WIDTH])
        height = int(stats[component, cv2.CC_STAT_HEIGHT])
        detections.append(ImageDetection(
            target.class_id,
            target.confidence,
            x_value,
            y_value,
            x_value + width - 1,
            y_value + height - 1,
        ))
    return detections


def localize_with_lidar(
    detection,
    points_lidar,
    intrinsic,
    camera_translation_from_lidar=(-0.13, 0.0, 0.10),
    pixel_margin=4.0,
):
    """
    Return the nearest LiDAR surface associated with an image box.

    Gazebo camera and LiDAR frames use x-forward, y-left, z-up axes here.
    """
    points = np.asarray(points_lidar, dtype=np.float64)
    matrix = np.asarray(intrinsic, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('points_lidar must have shape (N, 3)')
    if matrix.shape != (3, 3):
        raise ValueError('intrinsic must have shape (3, 3)')
    camera = points + np.asarray(camera_translation_from_lidar)
    forward = camera[:, 0]
    valid = np.isfinite(camera).all(axis=1) & (forward > 0.10)
    camera = camera[valid]
    source = points[valid]
    if not len(camera):
        return None
    fx, fy = matrix[0, 0], matrix[1, 1]
    cx, cy = matrix[0, 2], matrix[1, 2]
    pixels_x = cx - fx * camera[:, 1] / camera[:, 0]
    pixels_y = cy - fy * camera[:, 2] / camera[:, 0]
    inside = (
        (pixels_x >= detection.x_min - pixel_margin)
        & (pixels_x <= detection.x_max + pixel_margin)
        & (pixels_y >= detection.y_min - pixel_margin)
        & (pixels_y <= detection.y_max + pixel_margin)
    )
    candidates = source[inside]
    if len(candidates) < 2:
        return None
    ranges = np.linalg.norm(candidates, axis=1)
    nearest = float(np.percentile(ranges, 20.0))
    surface = candidates[ranges <= nearest + 0.35]
    if len(surface) < 2:
        surface = candidates[np.argsort(ranges)[:min(4, len(candidates))]]
    return np.median(surface, axis=0)


def lidar_point_to_map(point_lidar, robot_pose, lidar_offset=(0.20, 0.0, 0.45)):
    """Transform an x-forward LiDAR point to the map frame."""
    point = np.asarray(point_lidar, dtype=np.float64)
    if point.shape != (3,):
        raise ValueError('point_lidar must have shape (3,)')
    x_value, y_value, z_value, yaw = robot_pose
    base = point + np.asarray(lidar_offset, dtype=np.float64)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return np.asarray([
        x_value + cosine * base[0] - sine * base[1],
        y_value + sine * base[0] + cosine * base[1],
        z_value + base[2],
    ])
