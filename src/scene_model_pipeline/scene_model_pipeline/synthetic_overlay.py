# Copyright 2026 Kookmin AI Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Create deterministic route, communication, and event mock data."""

import argparse
import json
from pathlib import Path

from scene_model_pipeline.core import load_json


def write_synthetic_overlay(
    scene_manifest_path: Path,
    output_path: Path,
) -> dict:
    """Create an overlay aligned with a cumulative scene manifest."""
    scene = load_json(Path(scene_manifest_path))
    route = [
        [
            float(frame['map_from_lidar'][0][3]),
            float(frame['map_from_lidar'][1][3]),
            float(frame['map_from_lidar'][2][3]),
        ]
        for frame in scene['frames']
    ]
    timestamp_ns = max(
        int(frame['lidar_timestamp_ns']) for frame in scene['frames']
    )
    overlay = {
        'schema_version': 1,
        'mission_id': scene['mission_id'],
        'scene_id': scene['scene_id'],
        'coordinate_frame': scene['coordinate_frame'],
        'timestamp_ns': timestamp_ns,
        'route': {
            'id': 'actual_route_mock_001',
            'points': route,
        },
        'recovery_route': {
            'id': 'recovery_route_mock_001',
            'points': list(reversed(route[-3:])),
        },
        'recovery_waypoints': [
            {
                'id': 'crw_mock_001',
                'position': route[-2],
                'channel': 'mock_ethernet',
                'signal_strength_dbm': -48.0,
                'snr_db': 24.0,
                'packet_loss_ratio': 0.01,
                'latency_ms': 18.0,
                'safe_to_return': True,
                'route_edge_id': 'actual_route_mock_001',
                'selected_for_recovery': True,
                'timestamp_ns': timestamp_ns,
            },
        ],
        'communication_zones': [
            {
                'id': 'comm_zone_mock_001',
                'center': [route[-2][0], route[-2][1], route[-2][2]],
                'radius_m': 0.32,
                'classification': 'channel_anomaly_candidate',
                'confidence': 0.78,
                'source_log_id': 'mock_link_log_001',
            },
        ],
        'communication_summary': {
            'final_channel': 'mock_backup_wifi',
            'final_state': 'NORMAL',
            'state_sequence': [
                'NORMAL', 'STOPPING', 'RETURNING', 'CHANNEL_SWITCH',
                'SYNCING', 'REENTRY_TEST', 'NORMAL',
            ],
            'channel_switch_attempts': 2,
            'failure_cause': 2,
            'failure_confidence': 0.78,
        },
        'obstacles': [
            {
                'id': 'obstacle_mock_001',
                'position': [
                    route[1][0], route[1][1] + 0.32, route[1][2],
                ],
                'dimensions_m': [0.45, 0.45, 0.55],
                'classification': 'dynamic_obstacle_candidate',
                'label': 'Dynamic obstacle',
                'confidence': 0.93,
                'source_id': 'mock_lidar',
            },
        ],
        'mission_events': [
            {
                'id': 'event_mock_victim_001',
                'type': 'victim_candidate',
                'position': [0.15, 0.38, 3.18],
                'label': 'Victim candidate',
                'confidence': 0.82,
                'verified': False,
            },
            {
                'id': 'event_mock_hazard_001',
                'type': 'hazardous_material_candidate',
                'position': [-0.72, -0.32, 3.12],
                'label': 'Hazard candidate',
                'confidence': 0.74,
                'verified': False,
            },
        ],
    }
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open('w', encoding='utf-8') as stream:
        json.dump(overlay, stream, indent=2)
        stream.write('\n')
    return overlay


def main(args=None) -> None:
    """Write a mock overlay JSON for the generated cumulative model."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene-manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parsed = parser.parse_args(args)
    write_synthetic_overlay(parsed.scene_manifest, parsed.output)
    print('SYNTHETIC_OVERLAY=PASS')
    print(str(parsed.output))
