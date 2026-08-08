#!/usr/bin/env python3
"""Prove that mission, scene, overlay, and integrated MCAP share sources."""

import argparse
import hashlib
import json
from pathlib import Path


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def path_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    for item in sorted(value for value in path.rglob('*') if value.is_file()):
        digest.update(str(item.relative_to(path)).encode('utf-8'))
        digest.update(bytes.fromhex(file_sha256(item)))
    return digest.hexdigest()


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding='utf-8'))


def validate(mission: Path, scene: Path, integrated: Path) -> dict:
    mission_manifest_path = mission / 'replay_manifest.json'
    cumulative_path = scene / 'sequence_products/cumulative_manifest.json'
    overlay_path = scene / 'overlay_input/mission_overlay.json'
    command_path = scene / 'command_view/command_view_manifest.json'
    integrated_path = integrated / 'replay_manifest.json'
    mission_manifest = load(mission_manifest_path)
    cumulative = load(cumulative_path)
    overlay = load(overlay_path)
    command = load(command_path)
    combined = load(integrated_path)

    mission_hash = path_sha256(mission)
    if cumulative['bag']['sha256'] != mission_hash:
        raise ValueError('scene bag hash does not match mission recording')
    if overlay['sources']['bag_sha256'] != mission_hash:
        raise ValueError('overlay bag hash does not match mission recording')
    if command['sources']['scene_manifest']['sha256'] \
            != file_sha256(cumulative_path):
        raise ValueError('command view scene manifest hash is invalid')
    if command['sources']['overlay']['sha256'] != file_sha256(overlay_path):
        raise ValueError('command view overlay hash is invalid')
    sources = combined['sources']
    if sources['mission_manifest']['sha256'] \
            != file_sha256(mission_manifest_path):
        raise ValueError('integrated mission manifest hash is invalid')
    if sources['scene_manifest']['sha256'] != file_sha256(command_path):
        raise ValueError('integrated scene manifest hash is invalid')
    alignment = combined['time_alignment']
    if (
        alignment['mode'] != 'preserved_source_time'
        or int(alignment['scene_shift_ns']) != 0
    ):
        raise ValueError('scene timestamp was changed during integration')
    source_count = int(mission_manifest['message_count'])
    added_tf_count = (
        0 if mission_manifest['topic_counts'].get('/tf_static') else 1
    )
    if int(combined['message_count']) != source_count + 4 + added_tf_count:
        raise ValueError('integrated message count is not source + scene + TF')
    if cumulative['mission_id'] != overlay['mission_id']:
        raise ValueError('mission identity differs between scene and overlay')

    report = {
        'schema_version': 1,
        'verification': {
            'scene_uses_mission_bag_hash': True,
            'overlay_uses_mission_bag_hash': True,
            'command_view_hash_chain_valid': True,
            'integrated_hash_chain_valid': True,
            'scene_timestamp_preserved': True,
            'mission_identity_consistent': True,
        },
        'source_mcap': {
            'message_count': source_count,
            'camera_messages': mission_manifest['sensor_samples'][
                'camera_message_count'
            ],
            'lidar_messages': mission_manifest['sensor_samples'][
                'lidar_message_count'
            ],
            'sensor_sync_ratio': mission_manifest['sensor_samples'][
                'synchronized_lidar_ratio'
            ],
        },
        'scene': {
            'sampled_frames': cumulative['frame_count'],
            'usable_frames': cumulative['usable_frame_count'],
            'rejected_frames': cumulative['rejected_frame_count'],
            'colored_points': cumulative['products']['colored_cloud'][
                'point_count'
            ],
            'mesh_triangles': cumulative['products']['mesh'][
                'triangle_count'
            ],
        },
        'integrated_mcap': {
            'message_count': combined['message_count'],
            'duration_s': combined['duration_s'],
            'topic_count': len(combined['topic_counts']),
        },
    }
    output_path = integrated / 'lineage_validation.json'
    output_path.write_text(
        json.dumps(report, indent=2) + '\n', encoding='utf-8'
    )
    return report


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--mission', required=True, type=Path)
    parser.add_argument('--scene', required=True, type=Path)
    parser.add_argument('--integrated', required=True, type=Path)
    args = parser.parse_args()
    result = validate(args.mission, args.scene, args.integrated)
    print(
        'SAME_SOURCE_LINEAGE=PASS '
        f"messages={result['integrated_mcap']['message_count']} "
        f"topics={result['integrated_mcap']['topic_count']}"
    )


if __name__ == '__main__':
    main()
