# Copyright 2026 Kookmin AI Lab
# Licensed under the Apache License, Version 2.0

from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_vision60_model_has_physical_mass_envelope_and_sensors():
    root = ET.parse(
        PACKAGE_ROOT / 'models' / 'vision60' / 'model.sdf'
    ).getroot()
    model = root.find('model')
    assert model is not None and model.attrib['name'] == 'vision60'
    assert float(model.findtext('link/inertial/mass')) == 51.0
    body_size = model.findtext(
        "link/collision[@name='body_collision']/geometry/box/size"
    )
    assert body_size == '0.65 0.25 0.22'
    collisions = model.findall('link/collision')
    assert len(collisions) == 9
    sensor_types = {
        value.attrib['type'] for value in model.findall('link/sensor')
    }
    assert {'imu', 'gpu_lidar', 'camera'} <= sensor_types
    topics = {value.findtext('topic') for value in model.findall('link/sensor')}
    assert {'vision60/sim/imu', 'ouster', 'camera/image_raw'} <= topics
    plugins = {value.attrib['name'] for value in model.findall('plugin')}
    assert 'gz::sim::systems::VelocityControl' in plugins


def test_disaster_world_has_physics_and_repeatable_hazards():
    root = ET.parse(
        PACKAGE_ROOT / 'worlds' / 'disaster_test.sdf'
    ).getroot()
    world = root.find('world')
    assert world is not None
    assert float(world.findtext('physics/max_step_size')) == 0.001
    names = {value.attrib['name'] for value in world.findall('model')}
    assert {
        'ground_plane',
        'collapsed_wall_left',
        'collapsed_wall_right',
        'rubble_block_1',
        'rubble_block_2',
        'inclined_slab',
    } <= names
    included = world.find("include[name='vision60']/uri")
    assert included is not None and included.text == 'model://vision60'


def test_dynamic_world_isolated_from_static_rubble():
    root = ET.parse(
        PACKAGE_ROOT / 'worlds' / 'dynamic_obstacle_test.sdf'
    ).getroot()
    world = root.find('world')
    names = {value.attrib['name'] for value in world.findall('model')}
    assert {'ground_plane', 'corridor_left', 'corridor_right'} <= names
    assert not {'rubble_block_1', 'rubble_block_2'} & names
    included = world.find("include[name='vision60']/uri")
    assert included is not None and included.text == 'model://vision60'


def test_launch_bridges_existing_autonomy_sensor_contract():
    launch_text = (
        PACKAGE_ROOT / 'launch' / 'digital_twin.launch.py'
    ).read_text(encoding='utf-8')
    for topic in (
        '/clock',
        '/vision60/sim/imu',
        '/ouster/points',
        '/camera/image_raw',
        '/camera/camera_info',
        '/vision60/sim/poses',
    ):
        assert topic in launch_text
    assert 'pose_to_odometry' in launch_text
    assert 'body_twist_to_world' in launch_text
    adapter_text = (
        PACKAGE_ROOT / 'scripts' / 'body_twist_to_world'
    ).read_text(encoding='utf-8')
    assert '/cmd_vel_safe' in adapter_text
    assert '/vision60/sim/cmd_vel_world' in adapter_text


def test_obstacle_launch_uses_nav2_lidar_safety_chain():
    launch_text = (
        PACKAGE_ROOT / 'launch' / 'digital_twin_obstacle_avoidance.launch.py'
    ).read_text(encoding='utf-8')
    for executable in (
        'controller_server',
        'planner_server',
        'velocity_smoother',
        'collision_monitor',
        'safety_velocity_gate',
        'digital_twin_obstacle_probe',
    ):
        assert executable in launch_text
    config = (
        PACKAGE_ROOT / 'config' / 'nav2_obstacle_avoidance.yaml'
    ).read_text(encoding='utf-8')
    assert 'nav2_costmap_2d::VoxelLayer' in config
    assert 'nav2_costmap_2d::StaticLayer' in config
    assert 'nav2_navfn_planner/NavfnPlanner' in config
    assert 'RegulatedPurePursuitController' in config
    assert '/ouster/points' in config


def test_recovery_launch_uses_production_recovery_nodes():
    launch_text = (
        PACKAGE_ROOT / 'launch' / 'digital_twin_recovery.launch.py'
    ).read_text(encoding='utf-8')
    for executable in (
        'route_recorder',
        'comm_recovery_manager',
        'communication_channel_manager',
        'recovery_path_follower',
        'reentry_path_follower',
        'mission_logger',
        'digital_twin_recovery_harness',
    ):
        assert executable in launch_text


def test_integrated_launch_combines_nav2_and_recovery_chain():
    launch_text = (
        PACKAGE_ROOT
        / 'launch'
        / 'digital_twin_integrated_recovery.launch.py'
    ).read_text(encoding='utf-8')
    for executable in (
        'digital_twin_obstacle_avoidance.launch.py',
        'route_recorder',
        'comm_recovery_manager',
        'communication_channel_manager',
        'recovery_path_follower',
        'reentry_path_follower',
        'mission_logger',
        'digital_twin_integrated_recovery_harness',
    ):
        assert executable in launch_text
    assert "'require_motion_permission': 'true'" in launch_text


def test_frontier_launch_gates_exploration_during_recovery():
    launch_text = (
        PACKAGE_ROOT
        / 'launch'
        / 'digital_twin_frontier_exploration.launch.py'
    ).read_text(encoding='utf-8')
    for executable in (
        'explore_lite',
        'exploration_safety_gate',
        'bt_navigator',
        'route_recorder',
        'comm_recovery_manager',
        'recovery_path_follower',
        'reentry_path_follower',
        'digital_twin_frontier_harness',
    ):
        assert executable in launch_text
    config = (
        PACKAGE_ROOT / 'config' / 'frontier_exploration.yaml'
    ).read_text(encoding='utf-8')
    assert 'costmap_topic: /map' in config
    assert 'track_unknown_space: true' in config
    assert 'navigate_w_replanning_only_if_goal_is_updated.xml' in config


def test_perception_world_and_launch_have_fused_mission_targets():
    root = ET.parse(
        PACKAGE_ROOT / 'worlds' / 'perception_test.sdf'
    ).getroot()
    names = {
        value.attrib['name'] for value in root.find('world').findall('model')
    }
    assert {'victim_target', 'hazard_target'} <= names
    launch_text = (
        PACKAGE_ROOT / 'launch' / 'digital_twin_perception.launch.py'
    ).read_text(encoding='utf-8')
    assert 'mission_perception' in launch_text
    assert 'digital_twin_perception_harness' in launch_text
