#!/usr/bin/env python3
"""Create a deterministic, scene-cluster-safe TAO fire/smoke split."""

import argparse
from collections import Counter, defaultdict
import json
import os
from pathlib import Path


RATIOS = {'train': 0.70, 'val': 0.15, 'test': 0.15}
SOURCE_TO_TAO = {1: 0, 2: 1}
METRICS = ('images', 'fire_positive_images', 'smoke_positive_images', 'negative_images')


def _source_key(file_name):
    path = Path(file_name)
    parts = path.parts
    if parts and parts[0] == 'images':
        return str(Path(*parts[1:]))
    return str(path)


def prepare(source_root, audit_report, output_root):
    """Assign complete perceptual components to one split and rewrite COCO IDs."""
    source_root = Path(source_root).resolve()
    audit = json.loads(Path(audit_report).read_text())
    output_root = Path(output_root).resolve()
    output_annotations = output_root / 'annotations'
    output_annotations.mkdir(parents=True, exist_ok=True)
    for split in RATIOS:
        (output_root / 'images' / split).mkdir(parents=True, exist_ok=True)

    records = {}
    for original_split in ('train', 'val', 'test'):
        payload = json.loads(
            (source_root / 'annotations' / f'{original_split}.json').read_text()
        )
        image_by_id = {}
        for image in payload['images']:
            key = _source_key(image['file_name'])
            if key in records:
                raise ValueError(f'duplicate source image key: {key}')
            records[key] = {
                'image': dict(image),
                'annotations': [],
                'classes': set(),
                'original_split': original_split,
            }
            image_by_id[int(image['id'])] = key
        for annotation in payload['annotations']:
            category_id = int(annotation['category_id'])
            if category_id not in SOURCE_TO_TAO:
                continue
            key = image_by_id[int(annotation['image_id'])]
            records[key]['annotations'].append(dict(annotation))
            records[key]['classes'].add(SOURCE_TO_TAO[category_id])

    assigned_to_component = set()
    components = []
    strict = audit['strict_scene_clusters']
    for item in strict['components']:
        files = sorted(item['files'])
        unknown = [name for name in files if name not in records]
        if unknown:
            raise ValueError(f'audit references unknown files: {unknown[:3]}')
        overlap = assigned_to_component.intersection(files)
        if overlap:
            raise ValueError(f'overlapping scene components: {sorted(overlap)[:3]}')
        assigned_to_component.update(files)
        components.append(files)
    components.extend(
        [filename] for filename in sorted(records)
        if filename not in assigned_to_component
    )

    def metrics(files):
        output = Counter(images=len(files))
        for filename in files:
            classes = records[filename]['classes']
            output['fire_positive_images'] += int(0 in classes)
            output['smoke_positive_images'] += int(1 in classes)
            output['negative_images'] += int(not classes)
        return output

    component_metrics = [(files, metrics(files)) for files in components]
    totals = Counter()
    for _, values in component_metrics:
        totals.update(values)
    targets = {
        split: {metric: max(1.0, totals[metric] * ratio) for metric in METRICS}
        for split, ratio in RATIOS.items()
    }
    current = {split: Counter() for split in RATIOS}
    assigned_components = {split: [] for split in RATIOS}

    component_metrics.sort(
        key=lambda item: (
            -max(
                item[1][metric] / max(1, totals[metric])
                for metric in METRICS
            ),
            -len(item[0]),
            item[0][0],
        )
    )
    for files, values in component_metrics:
        def score(split):
            fill = [
                (current[split][metric] + values[metric])
                / targets[split][metric]
                for metric in METRICS
            ]
            over = sum(max(0.0, value - 1.0) ** 2 for value in fill)
            return (over * 20.0 + sum(value ** 2 for value in fill), fill[0], split)

        destination = min(RATIOS, key=score)
        assigned_components[destination].append(files)
        current[destination].update(values)

    assignment = {}
    for split, split_components in assigned_components.items():
        for files in split_components:
            for filename in files:
                assignment[filename] = split

    output_payloads = {
        split: {
            'images': [], 'annotations': [],
            'categories': [{'id': 0, 'name': 'fire'}, {'id': 1, 'name': 'smoke'}],
        }
        for split in RATIOS
    }
    image_ids = Counter()
    annotation_ids = Counter()
    for filename in sorted(records):
        split = assignment[filename]
        image_ids[split] += 1
        image = dict(records[filename]['image'])
        old_image_id = image['id']
        image['id'] = image_ids[split]
        materialized_name = (
            records[filename]['original_split'] + '__' + Path(filename).name
        )
        source_image = source_root / 'images' / filename
        destination_image = output_root / 'images' / split / materialized_name
        if destination_image.exists():
            if not os.path.samefile(source_image, destination_image):
                raise ValueError(f'conflicting output image: {destination_image}')
        else:
            os.link(source_image, destination_image)
        image['file_name'] = materialized_name
        image['original_split'] = records[filename]['original_split']
        output_payloads[split]['images'].append(image)
        for annotation in records[filename]['annotations']:
            annotation_ids[split] += 1
            converted = dict(annotation)
            converted['id'] = annotation_ids[split]
            converted['image_id'] = image_ids[split]
            converted['category_id'] = SOURCE_TO_TAO[int(annotation['category_id'])]
            converted['source_annotation_id'] = annotation['id']
            converted['source_image_id'] = old_image_id
            output_payloads[split]['annotations'].append(converted)

    for split, payload in output_payloads.items():
        (output_annotations / f'{split}.json').write_text(
            json.dumps(payload, separators=(',', ':')) + '\n'
        )
    (output_root / 'classmap.txt').write_text('fire\nsmoke\n')

    component_splits = {}
    for split, split_components in assigned_components.items():
        for files in split_components:
            for filename in files:
                component_splits[filename] = split
    leakage_errors = []
    for files in components:
        destinations = {component_splits[name] for name in files}
        if len(destinations) != 1:
            leakage_errors.append(files[:3])

    manifest = {
        'schema_version': 1,
        'purpose': 'TAO RT-DETR fire/smoke split with perceptual scene isolation',
        'source_dataset': str(source_root),
        'audit_report': str(Path(audit_report).resolve()),
        'images_copied': False,
        'image_materialization': 'hardlink',
        'assignment_method': 'deterministic greedy component stratification',
        'component_method': strict['method'],
        'component_thresholds': {
            'phash': audit['method']['max_hamming_distance'],
            'dhash': strict['dhash_max_hamming_distance'],
        },
        'input_images': len(records),
        'scene_components': len(components),
        'multi_image_components': strict['multi_image_components'],
        'component_leakage_errors': leakage_errors,
        'splits': {
            split: {
                'images': len(output_payloads[split]['images']),
                'annotations': len(output_payloads[split]['annotations']),
                'metrics': dict(current[split]),
                'component_count': len(assigned_components[split]),
            }
            for split in RATIOS
        },
    }
    (output_root / 'tao_dataset_manifest.json').write_text(
        json.dumps(manifest, indent=2) + '\n'
    )
    if leakage_errors:
        raise ValueError(f'component leakage remains: {leakage_errors[:3]}')
    return manifest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', required=True, type=Path)
    parser.add_argument('--audit-report', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(prepare(args.source, args.audit_report, args.output), indent=2))


if __name__ == '__main__':
    main()
