#!/usr/bin/env python3
"""Create a two-class TAO RT-DETR annotation view without copying images."""

import argparse
from collections import Counter
import json
from pathlib import Path


SOURCE_TO_TAO = {1: (0, 'fire'), 2: (1, 'smoke')}


def prepare(source_root, output_root):
    """Remap canonical fire/smoke category IDs into a compact two-class view."""
    source_root = Path(source_root).resolve()
    output_root = Path(output_root).resolve()
    annotation_dir = output_root / 'annotations'
    annotation_dir.mkdir(parents=True, exist_ok=True)
    summary = {}
    for split in ('train', 'val', 'test'):
        source_path = source_root / 'annotations' / f'{split}.json'
        payload = json.loads(source_path.read_text())
        image_ids = {int(item['id']) for item in payload['images']}
        annotations = []
        class_counts = Counter()
        for annotation in payload['annotations']:
            category_id = int(annotation['category_id'])
            if category_id not in SOURCE_TO_TAO:
                continue
            converted = dict(annotation)
            converted['category_id'] = SOURCE_TO_TAO[category_id][0]
            annotations.append(converted)
            class_counts[SOURCE_TO_TAO[category_id][1]] += 1
        if any(int(item['image_id']) not in image_ids for item in annotations):
            raise ValueError(f'{split}: annotation references a missing image')
        converted_payload = {
            key: value for key, value in payload.items()
            if key not in {'images', 'annotations', 'categories'}
        }
        converted_payload['images'] = [
            {**image, 'file_name': Path(image['file_name']).name}
            for image in payload['images']
        ]
        converted_payload['annotations'] = annotations
        converted_payload['categories'] = [
            {'id': 0, 'name': 'fire'},
            {'id': 1, 'name': 'smoke'},
        ]
        destination = annotation_dir / f'{split}.json'
        destination.write_text(json.dumps(converted_payload, separators=(',', ':')) + '\n')
        summary[split] = {
            'images': len(payload['images']),
            'annotations': len(annotations),
            'class_annotations': dict(sorted(class_counts.items())),
            'image_directory': str(source_root / 'images' / split),
            'annotation_file': str(destination),
        }
    (output_root / 'classmap.txt').write_text('fire\nsmoke\n')
    manifest = {
        'schema_version': 1,
        'purpose': 'NVIDIA TAO RT-DETR fire/smoke training view',
        'source_dataset': str(source_root),
        'images_copied': False,
        'category_mapping': {
            'canonical_1_fire': 'tao_0_fire',
            'canonical_2_smoke': 'tao_1_smoke',
        },
        'splits': summary,
    }
    (output_root / 'tao_dataset_manifest.json').write_text(
        json.dumps(manifest, indent=2) + '\n'
    )
    return manifest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(prepare(args.source, args.output), indent=2))


if __name__ == '__main__':
    main()
