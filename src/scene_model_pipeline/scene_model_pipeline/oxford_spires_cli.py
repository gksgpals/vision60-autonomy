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

"""Command-line conversion for the Oxford Spires public sample."""

import argparse
from pathlib import Path

from scene_model_pipeline.oxford_spires import (
    convert_oxford_spires_sample,
)


def main(args=None) -> None:
    """Convert selected Oxford files into the standard regression MCAP."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parsed = parser.parse_args(args)
    manifest = convert_oxford_spires_sample(
        parsed.input, parsed.output
    )
    print('OXFORD_SPIRES_CONVERSION=PASS')
    print(f"FRAMES={len(manifest['selected_frames'])}")
