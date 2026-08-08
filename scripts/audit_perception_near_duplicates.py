#!/usr/bin/env python3
"""Audit a canonical perception dataset for perceptual near duplicates."""

import argparse
from collections import Counter, defaultdict
from itertools import combinations, product
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageOps


def _load_upstream(source):
    source = Path(source).resolve()
    sys.path.insert(0, str(source))
    from imagededup.handlers.search.bktree import BKTree
    from imagededup.methods import DHash, PHash
    return PHash, DHash, BKTree


def _split_name(relative_name):
    parts = Path(relative_name).parts
    if parts and parts[0] in {'train', 'val', 'test'}:
        return parts[0]
    for split_name in ('train', 'val', 'test'):
        if Path(relative_name).name.startswith(split_name + '_'):
            return split_name
    raise ValueError(f'cannot determine split for {relative_name}')


def _rgb_mae(first_path, second_path):
    with Image.open(first_path) as first_image:
        first = np.asarray(
            ImageOps.fit(first_image.convert('RGB'), (64, 64)), dtype=np.float32
        )
    with Image.open(second_path) as second_image:
        second = np.asarray(
            ImageOps.fit(second_image.convert('RGB'), (64, 64)), dtype=np.float32
        )
    return float(np.mean(np.abs(first - second)) / 255.0)


def _make_contact_sheet(image_root, pairs, output_path, limit=24):
    selected = pairs[:limit]
    if not selected:
        return None
    panel_width, panel_height = 320, 210
    sheet = Image.new(
        'RGB', (panel_width * 2, panel_height * len(selected)), '#171717'
    )
    draw = ImageDraw.Draw(sheet)
    for row, pair in enumerate(selected):
        for column, key in enumerate(('first', 'second')):
            with Image.open(image_root / pair[key]) as source:
                panel = ImageOps.contain(source.convert('RGB'), (300, 160))
            x_value = column * panel_width + (panel_width - panel.width) // 2
            y_value = row * panel_height + 4
            sheet.paste(panel, (x_value, y_value))
            draw.text(
                (column * panel_width + 8, row * panel_height + 168),
                pair[key], fill='white',
            )
        draw.text(
            (8, row * panel_height + 188),
            f"pHash distance={pair['distance']}  RGB-MAE={pair['rgb_mae']:.4f}",
            fill='#7bdcff',
        )
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(output_path)
    return str(output_path)


def audit_dataset(
    dataset_root,
    upstream_source,
    threshold=6,
    workers=8,
    report_path=None,
    contact_sheet_path=None,
    review_pair_limit=240,
    dhash_threshold=6,
):
    """Generate PHash/BK-tree duplicate candidates without deleting data."""
    root = Path(dataset_root)
    image_root = root / 'images'
    PHash, DHash, BKTree = _load_upstream(upstream_source)
    phasher = PHash(verbose=True)
    encodings = phasher.encode_images(
        image_dir=image_root, recursive=True, num_enc_workers=workers
    )
    dhasher = DHash(verbose=True)
    dhash_encodings = dhasher.encode_images(
        image_dir=image_root, recursive=True, num_enc_workers=workers
    )
    if encodings.keys() != dhash_encodings.keys():
        raise ValueError('PHash and DHash image sets differ')
    groups = defaultdict(list)
    for filename, encoding in encodings.items():
        groups[encoding].append(filename)
    representative_map = {
        sorted(filenames)[0]: encoding
        for encoding, filenames in groups.items()
    }
    sys.setrecursionlimit(max(100000, len(representative_map) * 2))
    tree = BKTree(representative_map, phasher.hamming_distance)
    representative_by_hash = {
        encoding: representative for representative, encoding
        in representative_map.items()
    }
    candidate_hash_pairs = set()
    for encoding, representative in representative_by_hash.items():
        for match_name, distance in tree.search(encoding, tol=threshold):
            other_encoding = representative_map[match_name]
            pair = tuple(sorted((encoding, other_encoding)))
            if pair[0] != pair[1]:
                candidate_hash_pairs.add((pair[0], pair[1], distance))

    candidate_groups = []
    candidate_pair_count = 0
    cross_split_count = 0
    same_split_count = 0
    distance_counts = Counter()
    review_sources = []
    strict_parent = {filename: filename for filename in encodings}
    strict_candidate_count = 0
    strict_cross_count = 0
    strict_same_count = 0

    def find(filename):
        while strict_parent[filename] != filename:
            strict_parent[filename] = strict_parent[strict_parent[filename]]
            filename = strict_parent[filename]
        return filename

    def union(first, second):
        first_root = find(first)
        second_root = find(second)
        if first_root != second_root:
            strict_parent[second_root] = first_root

    def register_group(first_hash, second_hash, distance):
        nonlocal candidate_pair_count, cross_split_count, same_split_count
        nonlocal strict_candidate_count, strict_cross_count, strict_same_count
        first_files = sorted(groups[first_hash])
        second_files = (
            first_files if first_hash == second_hash
            else sorted(groups[second_hash])
        )
        first_counts = Counter(_split_name(name) for name in first_files)
        second_counts = Counter(_split_name(name) for name in second_files)
        if first_hash == second_hash:
            total = len(first_files) * (len(first_files) - 1) // 2
            same = sum(value * (value - 1) // 2 for value in first_counts.values())
            iterator = combinations(first_files, 2)
        else:
            total = len(first_files) * len(second_files)
            same = sum(
                first_counts[split] * second_counts[split]
                for split in ('train', 'val', 'test')
            )
            iterator = product(first_files, second_files)
        cross = total - same
        candidate_pair_count += total
        cross_split_count += cross
        same_split_count += same
        distance_counts[int(distance)] += total
        candidate_groups.append({
            'distance': int(distance),
            'pair_count': total,
            'cross_split_pair_count': cross,
            'same_split_pair_count': same,
            'first_split_counts': dict(sorted(first_counts.items())),
            'second_split_counts': dict(sorted(second_counts.items())),
            'first_examples': first_files[:5],
            'second_examples': second_files[:5],
        })
        strict_iterator = (
            combinations(first_files, 2)
            if first_hash == second_hash
            else product(first_files, second_files)
        )
        for first, second in strict_iterator:
            dhash_distance = dhasher.hamming_distance(
                dhash_encodings[first], dhash_encodings[second]
            )
            if dhash_distance > dhash_threshold:
                continue
            strict_candidate_count += 1
            if _split_name(first) == _split_name(second):
                strict_same_count += 1
            else:
                strict_cross_count += 1
            union(first, second)
        sampled = 0
        for first, second in iterator:
            if sampled >= 4:
                break
            first_split = _split_name(first)
            second_split = _split_name(second)
            review_sources.append((
                first_split == second_split,
                int(distance),
                min(first, second),
                max(first, second),
                first_split,
                second_split,
            ))
            sampled += 1

    for encoding, filenames in groups.items():
        if len(filenames) > 1:
            register_group(encoding, encoding, 0)
    for first_hash, second_hash, distance in candidate_hash_pairs:
        register_group(first_hash, second_hash, distance)

    review_sources.sort()
    pairs = []
    for _, distance, first, second, first_split, second_split in review_sources[:review_pair_limit]:
        pairs.append({
            'first': first,
            'second': second,
            'first_split': first_split,
            'second_split': second_split,
            'cross_split': first_split != second_split,
            'distance': distance,
            'rgb_mae': _rgb_mae(image_root / first, image_root / second),
        })
    pairs.sort(
        key=lambda item: (
            not item['cross_split'], item['distance'], item['rgb_mae'],
            item['first'], item['second'],
        )
    )
    candidate_groups.sort(
        key=lambda item: (
            item['cross_split_pair_count'] == 0,
            item['distance'],
            -item['pair_count'],
        )
    )
    strict_members = defaultdict(list)
    for filename in sorted(encodings):
        strict_members[find(filename)].append(filename)
    strict_components = []
    for filenames in strict_members.values():
        if len(filenames) <= 1:
            continue
        split_counts = Counter(_split_name(name) for name in filenames)
        strict_components.append({
            'size': len(filenames),
            'cross_split': len(split_counts) > 1,
            'split_counts': dict(sorted(split_counts.items())),
            'files': filenames,
        })
    strict_components.sort(
        key=lambda item: (not item['cross_split'], -item['size'], item['files'][0])
    )
    report = {
        'passed': True,
        'automatic_deletion_performed': False,
        'method': {
            'repository': 'https://github.com/idealo/imagededup',
            'revision': 'f0534a6ec10c02379c627696fbd486841068631c',
            'algorithm': 'PHash-64 + BK-tree',
            'max_hamming_distance': threshold,
            'rgb_mae_is_review_signal_only': True,
        },
        'encoded_images': len(encodings),
        'unique_perceptual_hashes': len(groups),
        'candidate_pairs': candidate_pair_count,
        'cross_split_candidate_pairs': cross_split_count,
        'same_split_candidate_pairs': same_split_count,
        'distance_histogram': {
            str(key): distance_counts[key] for key in sorted(distance_counts)
        },
        'candidate_hash_groups': len(candidate_groups),
        'review_pairs_limit': review_pair_limit,
        'review_pairs': pairs,
        'candidate_groups': candidate_groups,
        'strict_scene_clusters': {
            'method': 'PHash distance <= threshold AND DHash distance <= threshold',
            'dhash_max_hamming_distance': dhash_threshold,
            'candidate_pairs': strict_candidate_count,
            'cross_split_candidate_pairs': strict_cross_count,
            'same_split_candidate_pairs': strict_same_count,
            'multi_image_components': len(strict_components),
            'cross_split_components': sum(
                item['cross_split'] for item in strict_components
            ),
            'components': strict_components,
        },
    }
    if contact_sheet_path:
        report['contact_sheet'] = _make_contact_sheet(
            image_root, pairs, contact_sheet_path
        )
    if report_path:
        report_path = Path(report_path)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2) + '\n')
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dataset', required=True, type=Path)
    parser.add_argument('--imagededup-source', required=True, type=Path)
    parser.add_argument('--threshold', type=int, default=6)
    parser.add_argument('--workers', type=int, default=8)
    parser.add_argument('--report', required=True, type=Path)
    parser.add_argument('--contact-sheet', required=True, type=Path)
    parser.add_argument('--review-pair-limit', type=int, default=240)
    parser.add_argument('--dhash-threshold', type=int, default=6)
    args = parser.parse_args()
    report = audit_dataset(
        args.dataset,
        args.imagededup_source,
        threshold=args.threshold,
        workers=args.workers,
        report_path=args.report,
        contact_sheet_path=args.contact_sheet,
        review_pair_limit=args.review_pair_limit,
        dhash_threshold=args.dhash_threshold,
    )
    print(json.dumps({
        key: value for key, value in report.items()
        if key not in {'review_pairs', 'candidate_groups', 'strict_scene_clusters'}
    }, indent=2))


if __name__ == '__main__':
    main()
