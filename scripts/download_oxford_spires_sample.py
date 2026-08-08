#!/usr/bin/env python3
"""Download four Oxford Spires frames without fetching full archives."""

import argparse
from pathlib import Path

import requests
from remotezip import RemoteZip


DATASET_REVISION = '03f4382308333aa70c3253f12acd3fbf0c7c4a15'
TOOLS_COMMIT = 'b456e1e2f263a79c19b6ed4052390eba609011d4'
SEQUENCE = '2024-03-12-keble-college-01'
DATASET_ROOT = (
    'https://huggingface.co/datasets/ori-drs/'
    'oxford_spires_dataset/resolve'
)
TOOLS_ROOT = (
    'https://raw.githubusercontent.com/ori-drs/'
    'oxford_spires_dataset'
)
IMAGES = [
    'cam0/1710252326.145681006.jpg',
    'cam0/1710252326.545678106.jpg',
    'cam0/1710252326.945675206.jpg',
    'cam0/1710252327.745669406.jpg',
]
CLOUDS = [
    'lidar-clouds/1710252326.122305000.pcd',
    'lidar-clouds/1710252326.522260000.pcd',
    'lidar-clouds/1710252326.921302000.pcd',
    'lidar-clouds/1710252327.720689000.pcd',
]


def _write_response(url: str, path: Path) -> None:
    response = requests.get(url, timeout=120)
    response.raise_for_status()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.part')
    temporary.write_bytes(response.content)
    temporary.replace(path)


def _extract_members(url: str, members, output_dir: Path, buffer_mb: int):
    with RemoteZip(
        url,
        initial_buffer_size=buffer_mb * 1024 * 1024,
        timeout=120,
    ) as archive:
        for member in members:
            target = output_dir / member
            target.parent.mkdir(parents=True, exist_ok=True)
            temporary = target.with_suffix(target.suffix + '.part')
            temporary.write_bytes(archive.read(member))
            temporary.replace(target)


def download_sample(output_dir: Path) -> None:
    """Download selected archive members and official metadata."""
    output_dir = Path(output_dir)
    sequence_root = f'{DATASET_ROOT}/{DATASET_REVISION}/sequences/{SEQUENCE}'
    _extract_members(
        f'{sequence_root}/raw/images.zip?download=true',
        IMAGES,
        output_dir / 'images',
        8,
    )
    _extract_members(
        f'{sequence_root}/raw/lidar-clouds.zip?download=true',
        CLOUDS,
        output_dir / 'clouds',
        2,
    )
    _write_response(
        f'{sequence_root}/processed/vilens-slam/'
        'slam-poses.csv?download=true',
        output_dir / 'slam-poses.csv',
    )
    _write_response(
        f'{TOOLS_ROOT}/{TOOLS_COMMIT}/configs/sensor.yaml',
        output_dir / 'sensor.yaml',
    )
    _write_response(
        f'{TOOLS_ROOT}/{TOOLS_COMMIT}/LICENSE.md',
        output_dir / 'LICENSE.md',
    )
    files = [value for value in output_dir.rglob('*') if value.is_file()]
    total_bytes = sum(value.stat().st_size for value in files)
    print('OXFORD_SPIRES_DOWNLOAD=PASS')
    print(f'FILES={len(files)} BYTES={total_bytes}')


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parsed = parser.parse_args()
    download_sample(parsed.output)


if __name__ == '__main__':
    main()
