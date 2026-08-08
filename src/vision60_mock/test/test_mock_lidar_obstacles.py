from builtin_interfaces.msg import Time
import pytest

from vision60_mock.lidar_heartbeat import (
    obstacle_phase,
    base_to_lidar,
    camera_info_message,
    points_for_phase,
    static_frame_transforms,
    static_scene_points,
    synthetic_camera_rgb,
    world_to_base,
)


def test_obstacle_scenario_has_clear_slowdown_and_stop_phases():
    assert obstacle_phase(1.0, 2.0, 1.0, 4.0, 1.0) == 'clear'
    assert obstacle_phase(2.0, 2.0, 1.0, 4.0, 1.0) == 'slowdown'
    assert obstacle_phase(4.0, 2.0, 1.0, 4.0, 1.0) == 'stop'


def test_mock_obstacles_cross_humble_point_threshold():
    assert len(points_for_phase('clear')) == 0
    assert len(points_for_phase('slowdown')) > 3
    assert len(points_for_phase('stop')) > 3


def test_static_scene_is_dense_and_moves_with_robot_pose():
    world = static_scene_points()
    assert len(world) > 1000
    initial = world_to_base(world, 0.0, 0.0, 0.0)
    moved = world_to_base(world, 0.5, 0.0, 0.0)
    assert len(initial) > 1000
    assert min(point[0] for point in moved) \
        < min(point[0] for point in initial)


def test_synthetic_camera_has_expected_size_and_texture():
    image = synthetic_camera_rgb(160, 120)
    assert len(image) == 160 * 120 * 3
    assert len(set(image)) > 20


def test_calibration_and_mounting_are_self_consistent():
    stamp = Time()
    info = camera_info_message(stamp, 'camera_optical', 160, 120)
    assert list(info.k) == [
        110.0, 0.0, 80.0,
        0.0, 110.0, 60.0,
        0.0, 0.0, 1.0,
    ]
    transforms = static_frame_transforms(
        stamp, 'os_sensor', 'camera_optical'
    )
    pairs = {
        (value.header.frame_id, value.child_frame_id)
        for value in transforms
    }
    assert pairs == {
        ('map', 'odom'),
        ('base_link', 'os_sensor'),
        ('base_link', 'camera_optical'),
    }
    lidar_point = base_to_lidar([(0.4, 0.0, 0.3)])[0]
    assert lidar_point == pytest.approx((0.2, 0.0, -0.15))
