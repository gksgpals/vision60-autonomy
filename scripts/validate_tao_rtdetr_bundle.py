#!/usr/bin/env python3
"""Validate the local TAO dataset view and deployment contract without a GPU."""

import argparse
from collections import Counter
import json
from pathlib import Path


def validate(dataset_root, spec_path):
    dataset_root = Path(dataset_root).resolve()
    spec_path = Path(spec_path).resolve()
    errors = []
    split_summary = {}
    expected_categories = [(0, 'fire'), (1, 'smoke')]
    for split in ('train', 'val', 'test'):
        annotation_path = dataset_root / 'annotations' / f'{split}.json'
        try:
            payload = json.loads(annotation_path.read_text())
        except (OSError, json.JSONDecodeError) as error:
            errors.append(f'{split}: cannot read annotation JSON: {error}')
            continue
        categories = [(int(item['id']), item['name']) for item in payload['categories']]
        if categories != expected_categories:
            errors.append(f'{split}: categories must be {expected_categories}, got {categories}')
        image_ids = {int(image['id']) for image in payload['images']}
        class_counts = Counter()
        missing_images = 0
        invalid_boxes = 0
        for image in payload['images']:
            image_name = Path(image['file_name'])
            candidates = (
                dataset_root / 'images' / split / image_name,
                dataset_root.parent / 'images' / split / image_name,
                dataset_root.parent / 'images' / image_name,
            )
            if not any(path.is_file() for path in candidates):
                missing_images += 1
        for annotation in payload['annotations']:
            if int(annotation['image_id']) not in image_ids:
                errors.append(f"{split}: missing image id {annotation['image_id']}")
            category_id = int(annotation['category_id'])
            if category_id not in (0, 1):
                errors.append(f'{split}: unexpected category id {category_id}')
            class_counts[expected_categories[category_id][1]] += 1
            x_value, y_value, width, height = map(float, annotation['bbox'])
            if min(x_value, y_value, width, height) < 0 or width <= 0 or height <= 0:
                invalid_boxes += 1
        if missing_images:
            errors.append(f'{split}: {missing_images} image files are missing')
        if invalid_boxes:
            errors.append(f'{split}: {invalid_boxes} invalid boxes')
        split_summary[split] = {
            'images': len(payload['images']),
            'annotations': len(payload['annotations']),
            'class_annotations': dict(sorted(class_counts.items())),
        }
    spec = spec_path.read_text()
    required_fragments = (
        'backbone: resnet_18', 'num_classes: 2', 'precision: fp16',
        'input_width: 640', 'input_height: 384', 'opset_version: 17',
        'data_type: fp16',
    )
    for fragment in required_fragments:
        if fragment not in spec:
            errors.append(f'spec missing required setting: {fragment}')
    result = {
        'passed': not errors,
        'gpu_training_executed': False,
        'scope': 'dataset/schema/path and deployment-contract validation',
        'splits': split_summary,
        'errors': errors,
    }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', required=True, type=Path)
    parser.add_argument('--spec', required=True, type=Path)
    parser.add_argument('--report', required=True, type=Path)
    args = parser.parse_args()
    result = validate(args.dataset, args.spec)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
