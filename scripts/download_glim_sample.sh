#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
DATASET_ROOT="${PROJECT_ROOT}/datasets.nosync/glim"
ARCHIVE="${DATASET_ROOT}/os1_128_01_downsampled.tar.gz"
EXPECTED_MD5="75ef57e84010217ab8a5f8da511ff2ea"
URL="https://zenodo.org/api/records/7233945/files/os1_128_01_downsampled.tar.gz/content"

file_md5() {
  if command -v md5 >/dev/null 2>&1; then
    md5 -q "$1"
  else
    md5sum "$1" | awk '{print $1}'
  fi
}

mkdir -p "${DATASET_ROOT}"

if [[ ! -f "${ARCHIVE}" ]] \
  || [[ "$(file_md5 "${ARCHIVE}")" != "${EXPECTED_MD5}" ]]; then
  curl -L --fail --retry 5 --retry-all-errors \
    -o "${ARCHIVE}.partial" "${URL}"
  if [[ "$(file_md5 "${ARCHIVE}.partial")" != "${EXPECTED_MD5}" ]]; then
    echo "ERROR: GLIM sample checksum mismatch" >&2
    exit 1
  fi
  mv "${ARCHIVE}.partial" "${ARCHIVE}"
fi

tar -xzf "${ARCHIVE}" -C "${DATASET_ROOT}"
echo "GLIM_SAMPLE_READY=${DATASET_ROOT}/os1_128_01_downsampled"
