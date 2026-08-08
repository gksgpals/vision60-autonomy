#!/usr/bin/env python3
"""Create a redacted, checksummed Vision60 field-support bundle."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import re
import shutil
import tempfile
import zipfile


SECRET_LINE = re.compile(
    r'(?i)^(\s*[^#\n]*(?:password|passwd|psk|token|secret|private[_ -]?key)\s*[:=]\s*).*$'
)
SECRET_KEY = re.compile(r'(?i)(password|passwd|psk|token|secret|private[_ -]?key)')
TEXT_SUFFIXES = {'.json', '.yaml', '.yml', '.txt', '.log', '.md', '.csv'}


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def redact_text(text):
    """Remove common credentials while preserving the diagnostic key name."""
    return '\n'.join(
        SECRET_LINE.sub(r'\1[REDACTED]', line) for line in text.splitlines()
    ) + ('\n' if text.endswith('\n') else '')


def redact_json(value):
    """Recursively remove values whose JSON key looks credential-related."""
    if isinstance(value, dict):
        return {
            key: ('[REDACTED]' if SECRET_KEY.search(str(key)) else redact_json(item))
            for key, item in value.items()
        }
    if isinstance(value, list):
        return [redact_json(item) for item in value]
    return value


def create_bundle(output, inputs, metadata=None):
    """Copy readable inputs, redact text, hash contents, and zip atomically."""
    output = Path(output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    manifest_files = []
    with tempfile.TemporaryDirectory(prefix='vision60_support_') as temporary:
        root = Path(temporary) / 'vision60_support_bundle'
        files_root = root / 'files'
        files_root.mkdir(parents=True)
        for label, source in sorted(inputs.items()):
            source = Path(source).resolve()
            if not source.is_file():
                manifest_files.append({
                    'label': label, 'source': str(source), 'included': False,
                    'reason': 'missing',
                })
                continue
            safe_name = re.sub(r'[^A-Za-z0-9_.-]+', '_', label).strip('_')
            destination = files_root / f'{safe_name}{source.suffix}'
            if source.suffix.lower() in TEXT_SUFFIXES:
                text = source.read_text(encoding='utf-8', errors='replace')
                if source.suffix.lower() == '.json':
                    try:
                        cleaned = json.dumps(
                            redact_json(json.loads(text)), indent=2
                        ) + '\n'
                    except json.JSONDecodeError:
                        cleaned = redact_text(text)
                else:
                    cleaned = redact_text(text)
                destination.write_text(cleaned, encoding='utf-8')
                redacted = text != destination.read_text(encoding='utf-8')
            else:
                shutil.copy2(source, destination)
                redacted = False
            manifest_files.append({
                'label': label,
                'source': str(source),
                'archive_path': str(destination.relative_to(root)),
                'included': True,
                'redaction_applied': redacted,
                'bytes': destination.stat().st_size,
                'sha256': sha256_file(destination),
            })
        manifest = {
            'schema_version': 1,
            'created_at': datetime.now(timezone.utc).isoformat(),
            'system': {
                'platform': platform.platform(),
                'python': platform.python_version(),
            },
            'metadata': metadata or {},
            'files': manifest_files,
            'warning': 'Review the archive before sharing; credentials are never intentionally collected.',
        }
        manifest_path = root / 'support_bundle_manifest.json'
        manifest_path.write_text(json.dumps(manifest, indent=2) + '\n')
        temporary_zip = Path(temporary) / 'bundle.zip'
        with zipfile.ZipFile(
            temporary_zip, 'w', compression=zipfile.ZIP_DEFLATED
        ) as archive:
            for path in sorted(root.rglob('*')):
                if path.is_file():
                    archive.write(path, path.relative_to(root.parent))
        shutil.copy2(temporary_zip, output)
    return {
        'passed': True,
        'output': str(output),
        'bytes': output.stat().st_size,
        'sha256': sha256_file(output),
        'included_files': sum(item['included'] for item in manifest_files),
        'missing_files': sum(not item['included'] for item in manifest_files),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument(
        '--input', action='append', default=[], metavar='LABEL=PATH',
        help='File to include; repeat as needed.',
    )
    parser.add_argument('--mission-id', default='unknown')
    parser.add_argument('--robot-serial', default='unknown')
    args = parser.parse_args()
    inputs = {}
    for value in args.input:
        if '=' not in value:
            parser.error('--input must be LABEL=PATH')
        label, path = value.split('=', 1)
        inputs[label] = Path(path)
    result = create_bundle(
        args.output,
        inputs,
        {'mission_id': args.mission_id, 'robot_serial': args.robot_serial},
    )
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
