import cv2
import json
import numpy as np
import pytest

from mission_perception.core import (
    detect_simulation_targets,
    lidar_point_to_map,
    localize_with_lidar,
    normalize_external_detection,
)
from mission_perception.dataset import DatasetValidationError, validate_dataset
from mission_perception.dataset_cli import (
    generate_mock_dfire_source,
    generate_mock_dataset,
    import_dfire_dataset,
)


def test_detects_both_simulation_targets():
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    cv2.rectangle(image, (20, 25), (45, 95), (255, 0, 255), -1)
    cv2.rectangle(image, (100, 40), (130, 100), (0, 255, 255), -1)
    labels = {item.class_id for item in detect_simulation_targets(image)}
    assert labels == {'victim_candidate', 'hazard_candidate'}


def test_lidar_box_association_prefers_nearest_surface():
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    cv2.rectangle(image, (65, 40), (95, 90), (255, 0, 255), -1)
    detection = detect_simulation_targets(image)[0]
    intrinsic = np.array([[100.0, 0.0, 80.0], [0.0, 100.0, 60.0], [0.0, 0.0, 1.0]])
    points = np.array([
        [2.0, 0.0, 0.0], [2.1, 0.05, -0.05], [4.5, 0.0, 0.0],
        [3.0, 2.0, 0.0],
    ])
    localized = localize_with_lidar(detection, points, intrinsic)
    assert localized is not None
    assert localized[0] < 2.3


def test_lidar_to_map_uses_robot_yaw_and_mount_offset():
    mapped = lidar_point_to_map(
        np.array([1.0, 0.0, -0.2]),
        (2.0, 3.0, 0.55, np.pi / 2.0),
    )
    assert np.allclose(mapped, [2.0, 4.2, 0.8], atol=1e-6)


def test_external_detection_is_mapped_and_clipped():
    detection = normalize_external_detection(
        '0', 0.91, 5.0, 50.0, 20.0, 30.0, 160, 120,
        {'0': 'fire_candidate', '1': 'smoke_candidate'},
    )
    assert detection.class_id == 'fire_candidate'
    assert detection.x_min == 0
    assert detection.x_max == 15


def test_external_detection_rejects_unknown_or_low_confidence():
    class_map = {'0': 'fire_candidate'}
    assert normalize_external_detection(
        '7', 0.9, 50, 50, 20, 20, 160, 120, class_map
    ) is None
    assert normalize_external_detection(
        '0', 0.2, 50, 50, 20, 20, 160, 120, class_map
    ) is None


def test_training_fixture_validates_all_classes_and_splits(tmp_path):
    report = generate_mock_dataset(tmp_path)
    assert report['passed'] is True
    assert report['image_counts'] == {'train': 1, 'val': 1, 'test': 1}
    assert all(value == 3 for value in report['class_counts'].values())


def test_validator_rejects_source_group_split_leakage(tmp_path):
    generate_mock_dataset(tmp_path)
    val_path = tmp_path / 'annotations' / 'val.json'
    val = json.loads(val_path.read_text(encoding='utf-8'))
    val['images'][0]['source_group'] = 'train_sequence_001'
    val_path.write_text(json.dumps(val), encoding='utf-8')
    with pytest.raises(DatasetValidationError, match='source_group leakage'):
        validate_dataset(tmp_path)


def test_dfire_import_maps_smoke_and_fire_to_canonical_ids(tmp_path):
    source = tmp_path / 'source'
    output = tmp_path / 'output'
    generate_mock_dfire_source(source)
    report = import_dfire_dataset(source, output, '4bf9c31-test')
    assert report['passed'] is True
    assert report['required_categories'] == ['fire', 'smoke']
    assert report['class_counts']['fire'] == 1
    assert report['class_counts']['smoke'] == 2
    manifest = json.loads(
        (output / 'dataset_manifest.json').read_text(encoding='utf-8')
    )
    assert manifest['sources'][0]['rights_review_required'] is True


def test_dfire_import_rejects_unknown_class(tmp_path):
    source = tmp_path / 'source'
    generate_mock_dfire_source(source)
    (source / 'train' / 'labels' / 'train.txt').write_text(
        '2 0.5 0.5 0.2 0.2\n', encoding='utf-8'
    )
    with pytest.raises(ValueError, match='unknown D-Fire class 2'):
        import_dfire_dataset(source, tmp_path / 'output', 'test')


def test_dfire_hardlink_mode_preserves_distribution_provenance(tmp_path):
    source = tmp_path / 'source'
    output = tmp_path / 'output'
    generate_mock_dfire_source(source)
    import_dfire_dataset(
        source,
        output,
        'upstream-test',
        materialization='hardlink',
        distribution_url='https://example.test/dfire-repack',
        distribution_revision='distribution-v1',
    )
    source_image = source / 'train' / 'images' / 'train.jpg'
    imported_image = output / 'images' / 'train' / 'train.jpg'
    assert source_image.stat().st_ino == imported_image.stat().st_ino
    manifest = json.loads(
        (output / 'dataset_manifest.json').read_text(encoding='utf-8')
    )
    recorded = manifest['sources'][0]
    assert recorded['url'] == 'https://example.test/dfire-repack'
    assert recorded['revision'] == 'distribution-v1'
    assert recorded['upstream_revision'] == 'upstream-test'


def test_dfire_clip_policy_records_out_of_bounds_repairs(tmp_path):
    source = tmp_path / 'source'
    output = tmp_path / 'output'
    generate_mock_dfire_source(source)
    label = source / 'train' / 'labels' / 'train.txt'
    label.write_text(
        label.read_text(encoding='utf-8') + '1 0.99 0.5 0.04 0.2\n',
        encoding='utf-8',
    )
    report = import_dfire_dataset(
        source, output, 'test', box_policy='clip'
    )
    assert report['quality']['clipped_boxes'] == 1
    assert report['quality']['max_normalized_overflow'] == pytest.approx(0.01)
    assert report['quality']['min_retained_area_ratio'] == pytest.approx(0.75)


def test_dfire_drop_policy_records_zero_area_label(tmp_path):
    source = tmp_path / 'source'
    output = tmp_path / 'output'
    generate_mock_dfire_source(source)
    label = source / 'train' / 'labels' / 'train.txt'
    label.write_text(
        label.read_text(encoding='utf-8') + '0 0.5 0.5 0.0 0.2\n',
        encoding='utf-8',
    )
    report = import_dfire_dataset(
        source, output, 'test', invalid_box_policy='drop'
    )
    assert report['quality']['dropped_boxes'] == 1
    assert report['quality']['dropped_box_reasons'] == {
        'non_positive_size': 1,
    }
