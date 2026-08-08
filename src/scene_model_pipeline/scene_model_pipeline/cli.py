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

"""Command line entry point for offline 3D scene generation."""

import argparse
import json
from pathlib import Path
import sys

from scene_model_pipeline.core import run_pipeline


def build_parser() -> argparse.ArgumentParser:
    """Create command-line arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--points', type=Path, required=True)
    parser.add_argument('--image', type=Path, required=True)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--metadata', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--voxel-size', type=float, default=0.10)
    parser.add_argument('--mesh-depth', type=int, default=7)
    parser.add_argument('--max-time-delta-ms', type=float, default=50.0)
    return parser


def main(args=None) -> None:
    """Run the pipeline and return a process exit status."""
    parsed = build_parser().parse_args(args)
    try:
        manifest = run_pipeline(
            parsed.points,
            parsed.image,
            parsed.calibration,
            parsed.metadata,
            parsed.output,
            parsed.voxel_size,
            parsed.mesh_depth,
            parsed.max_time_delta_ms,
        )
    except Exception as error:
        print(f'SCENE_MODEL_PIPELINE=FAIL: {error}', file=sys.stderr)
        sys.exit(1)
    summary = {
        name: {
            key: value for key, value in product.items()
            if key.endswith('_count')
        }
        for name, product in manifest['products'].items()
    }
    print('SCENE_MODEL_PIPELINE=PASS')
    print(json.dumps(summary, sort_keys=True))
