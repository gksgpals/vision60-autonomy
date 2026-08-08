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

"""Command-line entry point for command-view generation."""

import argparse
from pathlib import Path

from scene_model_pipeline.overlay import build_command_view


def main(args=None) -> None:
    """Build command-center image and ROS replay products."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--scene-dir', type=Path, required=True)
    parser.add_argument('--overlay', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--width', type=int, default=1280)
    parser.add_argument('--height', type=int, default=720)
    parsed = parser.parse_args(args)
    manifest = build_command_view(
        parsed.scene_dir,
        parsed.overlay,
        parsed.output,
        parsed.width,
        parsed.height,
    )
    print('COMMAND_VIEW=PASS')
    print(f"ROS_MARKERS={manifest['counts']['ros_markers']}")
