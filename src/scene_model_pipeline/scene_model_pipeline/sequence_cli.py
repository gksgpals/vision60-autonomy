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

"""CLI for generating a cumulative scene from a rosbag2 recording."""

import argparse
import json
from pathlib import Path
import sys

from scene_model_pipeline.sequence import build_cumulative_scene


def main(args=None) -> None:
    """Build a cumulative map and return a process exit status."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', type=Path, required=True)
    calibration = parser.add_mutually_exclusive_group(required=True)
    calibration.add_argument('--calibration', type=Path)
    calibration.add_argument('--calibration-from-bag', action='store_true')
    parser.add_argument('--metadata', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--voxel-size', type=float, default=0.10)
    parser.add_argument('--mesh-depth', type=int, default=7)
    parser.add_argument('--max-frames', type=int, default=100)
    parser.add_argument('--image-tolerance-ms', type=float, default=50.0)
    parser.add_argument('--pose-tolerance-ms', type=float, default=100.0)
    parser.add_argument('--frame-interval-ms', type=float, default=0.0)
    parser.add_argument('--lidar-topic', default='/ouster/points')
    parser.add_argument('--image-topic', default='/camera/image_raw')
    parser.add_argument('--pose-topic', default='/slam/odom')
    parser.add_argument('--storage-id', default='mcap')
    parsed = parser.parse_args(args)
    try:
        manifest = build_cumulative_scene(
            parsed.bag,
            parsed.calibration,
            parsed.metadata,
            parsed.output,
            parsed.voxel_size,
            parsed.mesh_depth,
            parsed.max_frames,
            parsed.image_tolerance_ms,
            parsed.pose_tolerance_ms,
            parsed.lidar_topic,
            parsed.image_topic,
            parsed.pose_topic,
            parsed.storage_id,
            parsed.frame_interval_ms,
        )
    except Exception as error:
        print(f'CUMULATIVE_SCENE=FAIL: {error}', file=sys.stderr)
        sys.exit(1)
    summary = {
        'frame_count': manifest['frame_count'],
        'usable_frame_count': manifest['usable_frame_count'],
        'rejected_frame_count': manifest['rejected_frame_count'],
        'products': {
            name: {
                key: value for key, value in product.items()
                if key.endswith('_count')
            }
            for name, product in manifest['products'].items()
        },
    }
    print('CUMULATIVE_SCENE=PASS')
    print(json.dumps(summary, sort_keys=True))
