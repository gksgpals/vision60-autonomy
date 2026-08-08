"""Command-line tools for reproducible perception datasets."""

import argparse
import json
import math
import os
from pathlib import Path
import shutil

import cv2
import numpy as np

from mission_perception.dataset import (
    PERCEPTION_CATEGORIES,
    file_sha256,
    validate_dataset,
)


def _categories():
    return [
        {'id': index, 'name': name}
        for index, name in enumerate(PERCEPTION_CATEGORIES)
    ]


def generate_mock_dataset(output_dir):
    """Create a tiny complete fixture that exercises the real validator."""
    root = Path(output_dir)
    (root / 'images').mkdir(parents=True, exist_ok=True)
    (root / 'annotations').mkdir(parents=True, exist_ok=True)
    splits = {}
    for split_index, split_name in enumerate(('train', 'val', 'test')):
        image = np.full(
            (180, 320, 3),
            (25 + split_index * 15, 30, 35),
            dtype=np.uint8,
        )
        images = []
        annotations = []
        image_id = split_index + 1
        for category_id, name in enumerate(PERCEPTION_CATEGORIES):
            x_value = 10 + category_id * 50
            y_value = 45 + (category_id % 2) * 50
            width, height = 35, 35
            color = (
                int(35 + category_id * 30),
                int(220 - category_id * 20),
                int(60 + split_index * 50),
            )
            cv2.rectangle(
                image, (x_value, y_value),
                (x_value + width, y_value + height), color, -1,
            )
            cv2.putText(
                image, name[:3], (x_value, y_value - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.25, (235, 235, 235), 1,
            )
            annotations.append({
                'id': split_index * 100 + category_id,
                'image_id': image_id,
                'category_id': category_id,
                'bbox': [x_value, y_value, width, height],
                'area': width * height,
                'iscrowd': 0,
            })
        relative_path = Path('images') / f'{split_name}_fixture.png'
        image_path = root / relative_path
        if not cv2.imwrite(str(image_path), image):
            raise RuntimeError(f'could not write {image_path}')
        images.append({
            'id': image_id,
            'file_name': str(relative_path),
            'width': 320,
            'height': 180,
            'sha256': file_sha256(image_path),
            'source_id': 'digital_twin_fixture',
            'source_group': f'{split_name}_sequence_001',
        })
        coco = {
            'images': images,
            'annotations': annotations,
            'categories': _categories(),
        }
        annotation_path = Path('annotations') / f'{split_name}.json'
        (root / annotation_path).write_text(
            json.dumps(coco, indent=2) + '\n', encoding='utf-8'
        )
        splits[split_name] = {'annotations': str(annotation_path)}
    manifest = {
        'schema_version': 1,
        'dataset_id': 'vision60_perception_fixture_v1',
        'categories': _categories(),
        'splits': splits,
        'sources': [{
            'source_id': 'digital_twin_fixture',
            'name': 'Vision60 deterministic digital twin fixture',
            'url': 'local://vision60_simulation/perception_test.sdf',
            'revision': 'workspace-test-fixture-v1',
            'license': 'Apache-2.0',
            'viewpoint': 'robot-mounted RGB camera',
        }],
    }
    (root / 'dataset_manifest.json').write_text(
        json.dumps(manifest, indent=2) + '\n', encoding='utf-8'
    )
    return validate_dataset(root)


def _prepare_empty_output(root):
    if root.exists() and any(root.iterdir()):
        raise ValueError(f'output directory is not empty: {root}')
    (root / 'images').mkdir(parents=True, exist_ok=True)
    (root / 'annotations').mkdir(parents=True, exist_ok=True)


def generate_mock_dfire_source(output_dir):
    """Create a tiny official-layout D-Fire source for integration tests."""
    root = Path(output_dir)
    if root.exists() and any(root.iterdir()):
        raise ValueError(f'output directory is not empty: {root}')
    labels = {
        'train': '0 0.5 0.5 0.4 0.4\n1 0.25 0.25 0.2 0.2\n',
        'val': '0 0.5 0.5 0.2 0.4\n',
        'test': '',
    }
    for split_index, split_name in enumerate(('train', 'val', 'test')):
        image_dir = root / split_name / 'images'
        label_dir = root / split_name / 'labels'
        image_dir.mkdir(parents=True, exist_ok=True)
        label_dir.mkdir(parents=True, exist_ok=True)
        image = np.full(
            (100, 200, 3),
            (30 + split_index * 20, 40, 50),
            dtype=np.uint8,
        )
        image_path = image_dir / f'{split_name}.jpg'
        if not cv2.imwrite(str(image_path), image):
            raise RuntimeError(f'could not write {image_path}')
        (label_dir / f'{split_name}.txt').write_text(
            labels[split_name], encoding='utf-8'
        )
    return root


def _dfire_box(
    parts,
    image_width,
    image_height,
    label_path,
    line_number,
    box_policy='strict',
    invalid_box_policy='reject',
):
    if len(parts) != 5:
        raise ValueError(
            f'{label_path}:{line_number}: expected 5 YOLO fields'
        )
    try:
        source_class = int(parts[0])
        center_x, center_y, width, height = map(float, parts[1:])
    except ValueError as error:
        raise ValueError(
            f'{label_path}:{line_number}: non-numeric YOLO label'
        ) from error
    class_map = {0: 2, 1: 1}  # D-Fire smoke/fire -> canonical IDs.
    if source_class not in class_map:
        raise ValueError(
            f'{label_path}:{line_number}: unknown D-Fire class '
            f'{source_class}'
        )
    values = (center_x, center_y, width, height)
    if invalid_box_policy not in {'reject', 'drop'}:
        raise ValueError('invalid_box_policy must be reject or drop')
    invalid_reason = None
    if not all(math.isfinite(value) for value in values):
        invalid_reason = 'non_finite'
    elif width <= 0.0 or height <= 0.0:
        invalid_reason = 'non_positive_size'
    if invalid_reason:
        if invalid_box_policy == 'drop':
            return class_map[source_class], None, {
                'clipped': False,
                'dropped': True,
                'drop_reason': invalid_reason,
            }
        raise ValueError(
            f'{label_path}:{line_number}: invalid YOLO box: {invalid_reason}'
        )
    x_min = center_x - width / 2.0
    y_min = center_y - height / 2.0
    x_max = center_x + width / 2.0
    y_max = center_y + height / 2.0
    overflow = max(-x_min, -y_min, x_max - 1.0, y_max - 1.0, 0.0)
    if box_policy == 'strict' and overflow > 1e-9:
        raise ValueError(
            f'{label_path}:{line_number}: YOLO box is outside the image'
        )
    if box_policy not in {'strict', 'clip'}:
        raise ValueError('box_policy must be strict or clip')
    clipped_x_min = max(0.0, x_min)
    clipped_y_min = max(0.0, y_min)
    clipped_x_max = min(1.0, x_max)
    clipped_y_max = min(1.0, y_max)
    clipped_width = clipped_x_max - clipped_x_min
    clipped_height = clipped_y_max - clipped_y_min
    if clipped_width <= 0.0 or clipped_height <= 0.0:
        if invalid_box_policy == 'drop':
            return class_map[source_class], None, {
                'clipped': False,
                'dropped': True,
                'drop_reason': 'no_image_intersection',
            }
        raise ValueError(
            f'{label_path}:{line_number}: YOLO box does not intersect image'
        )
    retained_ratio = (clipped_width * clipped_height) / (width * height)
    return class_map[source_class], [
        clipped_x_min * image_width,
        clipped_y_min * image_height,
        clipped_width * image_width,
        clipped_height * image_height,
    ], {
        'clipped': overflow > 1e-9,
        'dropped': False,
        'normalized_overflow': overflow,
        'retained_area_ratio': retained_ratio,
    }


def import_dfire_dataset(
    source_dir,
    output_dir,
    revision,
    materialization='copy',
    distribution_url=None,
    distribution_revision=None,
    box_policy='strict',
    invalid_box_policy='reject',
):
    """Convert an official D-Fire YOLO split tree to canonical COCO shards."""
    source = Path(source_dir)
    root = Path(output_dir)
    _prepare_empty_output(root)
    if materialization not in {'copy', 'hardlink'}:
        raise ValueError('materialization must be copy or hardlink')
    if box_policy not in {'strict', 'clip'}:
        raise ValueError('box_policy must be strict or clip')
    if invalid_box_policy not in {'reject', 'drop'}:
        raise ValueError('invalid_box_policy must be reject or drop')
    quality = {
        'box_policy': box_policy,
        'invalid_box_policy': invalid_box_policy,
        'clipped_boxes': 0,
        'dropped_boxes': 0,
        'dropped_box_reasons': {},
        'max_normalized_overflow': 0.0,
        'min_retained_area_ratio': 1.0,
    }
    splits = {}
    next_image_id = 1
    next_annotation_id = 1
    image_extensions = {'.jpg', '.jpeg', '.png', '.bmp', '.webp'}
    for split_name in ('train', 'val', 'test'):
        image_root = source / split_name / 'images'
        label_root = source / split_name / 'labels'
        if not image_root.is_dir() or not label_root.is_dir():
            raise ValueError(
                f'D-Fire split requires {image_root} and {label_root}'
            )
        images = []
        annotations = []
        image_paths = sorted(
            path for path in image_root.rglob('*')
            if path.is_file() and path.suffix.lower() in image_extensions
        )
        if not image_paths:
            raise ValueError(f'D-Fire split has no images: {image_root}')
        for source_image in image_paths:
            relative = source_image.relative_to(image_root)
            label_path = (label_root / relative).with_suffix('.txt')
            if not label_path.is_file():
                raise ValueError(f'missing D-Fire label file: {label_path}')
            decoded = cv2.imread(str(source_image))
            if decoded is None:
                raise ValueError(f'cannot decode D-Fire image: {source_image}')
            image_height, image_width = decoded.shape[:2]
            destination = root / 'images' / split_name / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            if materialization == 'hardlink':
                os.link(source_image, destination)
            else:
                shutil.copy2(source_image, destination)
            image_id = next_image_id
            next_image_id += 1
            images.append({
                'id': image_id,
                'file_name': str(destination.relative_to(root)),
                'width': image_width,
                'height': image_height,
                'sha256': file_sha256(destination),
                'source_id': 'dfire',
                'source_group': 'dfire/' + str(relative.with_suffix('')),
            })
            lines = label_path.read_text(encoding='utf-8').splitlines()
            for line_number, line in enumerate(lines, start=1):
                if not line.strip():
                    continue
                category_id, bbox, box_quality = _dfire_box(
                    line.split(), image_width, image_height,
                    label_path, line_number, box_policy, invalid_box_policy,
                )
                if box_quality['dropped']:
                    reason = box_quality['drop_reason']
                    quality['dropped_boxes'] += 1
                    quality['dropped_box_reasons'][reason] = (
                        quality['dropped_box_reasons'].get(reason, 0) + 1
                    )
                    continue
                if box_quality['clipped']:
                    quality['clipped_boxes'] += 1
                    quality['max_normalized_overflow'] = max(
                        quality['max_normalized_overflow'],
                        box_quality['normalized_overflow'],
                    )
                    quality['min_retained_area_ratio'] = min(
                        quality['min_retained_area_ratio'],
                        box_quality['retained_area_ratio'],
                    )
                annotations.append({
                    'id': next_annotation_id,
                    'image_id': image_id,
                    'category_id': category_id,
                    'bbox': bbox,
                    'area': bbox[2] * bbox[3],
                    'iscrowd': 0,
                })
                next_annotation_id += 1
        coco = {
            'images': images,
            'annotations': annotations,
            'categories': _categories(),
        }
        annotation_path = Path('annotations') / f'{split_name}.json'
        (root / annotation_path).write_text(
            json.dumps(coco, indent=2) + '\n', encoding='utf-8'
        )
        splits[split_name] = {'annotations': str(annotation_path)}
    manifest = {
        'schema_version': 1,
        'dataset_id': f'dfire_{revision[:12]}',
        'categories': _categories(),
        'required_categories': ['fire', 'smoke'],
        'quality': quality,
        'splits': splits,
        'sources': [{
            'source_id': 'dfire',
            'name': 'D-Fire Dataset',
            'url': distribution_url or (
                'https://github.com/gaia-solutions-on-demand/DFireDataset'
            ),
            'revision': distribution_revision or revision,
            'upstream_url': (
                'https://github.com/gaia-solutions-on-demand/DFireDataset'
            ),
            'upstream_revision': revision,
            'materialization': materialization,
            'license': 'CC0-1.0 collection; source-image rights review required',
            'viewpoint': 'mixed fixed and handheld RGB cameras',
            'rights_review_required': True,
            'original_class_map': {'0': 'smoke', '1': 'fire'},
        }],
    }
    (root / 'dataset_manifest.json').write_text(
        json.dumps(manifest, indent=2) + '\n', encoding='utf-8'
    )
    return validate_dataset(root)


def validation_main():
    """Validate a dataset and optionally write the machine-readable report."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', required=True, type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    report = validate_dataset(args.dataset)
    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(rendered + '\n', encoding='utf-8')
    print('PERCEPTION_DATASET_VALIDATION=PASS')
    print(rendered)


def fixture_main():
    """Generate and immediately validate a deterministic dataset fixture."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    report = generate_mock_dataset(args.output)
    print('PERCEPTION_DATASET_FIXTURE=PASS')
    print(json.dumps(report, ensure_ascii=False, indent=2))


def dfire_import_main():
    """Import and validate the official pre-split D-Fire YOLO dataset."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--revision', required=True)
    parser.add_argument(
        '--materialization', choices=('copy', 'hardlink'), default='copy'
    )
    parser.add_argument('--distribution-url')
    parser.add_argument('--distribution-revision')
    parser.add_argument(
        '--box-policy', choices=('strict', 'clip'), default='strict'
    )
    parser.add_argument(
        '--invalid-box-policy', choices=('reject', 'drop'), default='reject'
    )
    args = parser.parse_args()
    report = import_dfire_dataset(
        args.source,
        args.output,
        args.revision,
        materialization=args.materialization,
        distribution_url=args.distribution_url,
        distribution_revision=args.distribution_revision,
        box_policy=args.box_policy,
        invalid_box_policy=args.invalid_box_policy,
    )
    print('DFIRE_DATASET_IMPORT=PASS')
    print(json.dumps(report, ensure_ascii=False, indent=2))


def dfire_fixture_main():
    """Generate a deterministic D-Fire-layout source fixture."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    generate_mock_dfire_source(args.output)
    print('DFIRE_SOURCE_FIXTURE=PASS')
