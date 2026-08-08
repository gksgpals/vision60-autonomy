#!/usr/bin/env python3
"""Dependency-free regression test for support-bundle redaction and hashing."""

import json
from pathlib import Path
import tempfile
import zipfile

from create_support_bundle import create_bundle


def main():
    with tempfile.TemporaryDirectory(prefix='vision60_bundle_test_') as temporary:
        root = Path(temporary)
        diagnostic = root / 'diagnostics.yaml'
        diagnostic.write_text(
            'robot: vision60\npassword: do-not-package\npsk=also-secret\n'
        )
        json_diagnostic = root / 'network.json'
        json_diagnostic.write_text(
            '{"nested":{"api_token":"json-secret"},"status":"ok"}'
        )
        output = root / 'support.zip'
        result = create_bundle(
            output,
            {
                'diagnostics': diagnostic,
                'network': json_diagnostic,
                'missing_health': root / 'missing.json',
            },
            {'mission_id': 'mock-001'},
        )
        assert result['included_files'] == 2
        assert result['missing_files'] == 1
        with zipfile.ZipFile(output) as archive:
            names = archive.namelist()
            content = archive.read(
                'vision60_support_bundle/files/diagnostics.yaml'
            ).decode()
            manifest = json.loads(archive.read(
                'vision60_support_bundle/support_bundle_manifest.json'
            ))
            json_content = archive.read(
                'vision60_support_bundle/files/network.json'
            ).decode()
        assert 'do-not-package' not in content
        assert 'also-secret' not in content
        assert content.count('[REDACTED]') == 2
        assert 'json-secret' not in json_content
        assert len(manifest['files'][0]['sha256']) == 64
        assert any(name.endswith('support_bundle_manifest.json') for name in names)
    print('VISION60_SUPPORT_BUNDLE=PASS')


if __name__ == '__main__':
    main()
