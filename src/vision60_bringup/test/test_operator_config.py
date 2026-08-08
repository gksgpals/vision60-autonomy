import json
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_ROOT = PACKAGE_ROOT / 'config'


def _display_topics(config):
    displays = config['Visualization Manager']['Displays']
    return {
        display.get('Topic', {}).get('Value')
        for display in displays
        if isinstance(display.get('Topic'), dict)
    }


def test_bridge_is_loopback_read_only_and_filtered():
    config = yaml.safe_load(
        (CONFIG_ROOT / 'foxglove_bridge.yaml').read_text()
    )['foxglove_bridge']['ros__parameters']

    assert config['address'] == '127.0.0.1'
    assert config['port'] == 8765
    assert config['capabilities'] == ['connectionGraph']
    assert config['client_topic_whitelist'] == ['(?!)']
    assert config['service_whitelist'] == ['(?!)']
    assert config['param_whitelist'] == ['(?!)']
    assert config['asset_uri_allowlist'] == ['(?!)']
    assert '.*' not in config['topic_whitelist']


def test_rviz_contains_mission_scene_and_recovery_paths():
    config = yaml.safe_load((CONFIG_ROOT / 'operator.rviz').read_text())
    assert config['Visualization Manager']['Global Options']['Fixed Frame'] \
        == 'map'
    topics = _display_topics(config)
    assert '/mission/scene_cloud' in topics
    assert '/mission/scene_markers' in topics
    assert '/mission/scene_mesh' in topics
    assert '/mission/voxel_markers' in topics
    assert '/mission/recorded_path' in topics
    assert '/mission/recovery_path' in topics
    assert '/vision60/odom' in topics


def test_operator_contract_covers_scene_camera_link_and_safety():
    config = yaml.safe_load(
        (CONFIG_ROOT / 'operator_topics.yaml').read_text()
    )
    assert config['display_frame'] == 'map'
    assert config['views']['camera']['topic'] == '/camera/image_raw'
    assert config['views']['camera']['calibration_topic'] \
        == '/camera/camera_info'
    assert config['views']['camera']['transform_topic'] == '/tf_static'
    assert '/mission/scene_cloud' in config['views']['scene_3d']['topics']
    assert '/mission/scene_mesh' in config['views']['scene_3d']['topics']
    assert '/mission/voxel_markers' in config['views']['scene_3d']['topics']
    assert '/communication/recovery_status' in \
        config['views']['recovery']['topics']
    assert '/vision60/safety_state' in \
        config['views']['recovery']['topics']
    assert '/vision60/mock_obstacle_phase' in \
        config['views']['collision']['topics']
    assert config['offline_replays']['integrated_operator_bag'] \
        == 'integrated_calibrated_replay'
    assert config['offline_replays']['full_system_recovery_bag'] \
        == 'full_system_calibrated_mock_replay'
    fields = config['views']['link_quality']['fields']
    assert '/communication/state.packet_loss_ratio' in fields
    assert '/communication/state.latency_ms' in fields


def test_foxglove_layout_covers_operator_mock_workflow():
    layout = json.loads(
        (
            CONFIG_ROOT
            / 'foxglove'
            / 'vision60_operator_layout.json'
        ).read_text()
    )
    configs = layout['configById']

    panel_types = {panel_id.split('!', 1)[0] for panel_id in configs}
    assert {'3D', 'Image', 'Plot', 'RawMessages', 'StateTransitions'} \
        <= panel_types

    serialized = json.dumps(layout)
    required_paths = {
        '/mission/recorded_path',
        '/mission/recovery_path',
        '/mission/scene_mesh',
        '/mission/voxel_markers',
        '/ouster/points',
        '/communication/state.signal_strength_dbm',
        '/communication/state.snr_db',
        '/communication/state.packet_loss_ratio',
        '/communication/state.latency_ms',
        '/communication/recovery_status.state',
        '/mission/sync_status.state',
        '/vision60/safety_state.motion_allowed',
        '/vision60/mock_obstacle_phase',
        '/mission/event',
        '/camera/image_raw',
    }
    for path in required_paths:
        assert path in serialized


def test_recovery_followers_use_registered_nav2_plugins():
    config = yaml.safe_load((CONFIG_ROOT / 'nav2_controller.yaml').read_text())
    controller = config['controller_server']['ros__parameters']
    controller_id = controller['controller_plugins'][0]
    goal_checker_id = controller['goal_checker_plugins'][0]
    source_root = PACKAGE_ROOT.parent / 'comm_recovery_manager' \
        / 'comm_recovery_manager'
    for filename in ('path_follower.py', 'reentry_follower.py'):
        source = (source_root / filename).read_text()
        assert f"declare_parameter('controller_id', '{controller_id}')" in source
        assert f"declare_parameter('goal_checker_id', '{goal_checker_id}')" in source


def test_rtdetr_fusion_is_fail_filtered_and_candidate_only():
    config = yaml.safe_load(
        (CONFIG_ROOT / 'mission_perception_rtdetr.yaml').read_text()
    )['mission_perception']['ros__parameters']
    class_map = json.loads(config['external_class_map_json'])
    assert config['detector_backend'] == 'external_vision_msgs'
    assert class_map == {
        '0': 'fire_candidate',
        '1': 'smoke_candidate',
    }
    assert config['minimum_confidence'] >= 0.35
    assert config['maximum_detection_age_s'] <= 0.25
