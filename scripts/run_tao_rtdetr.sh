#!/usr/bin/env bash
set -euo pipefail

ACTION="${1:-dry-run}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATASET="${VISION60_TAO_DATASET:-$ROOT/datasets.nosync/dfire/coco}"
RESULTS="${VISION60_TAO_RESULTS:-$ROOT/training/tao_rtdetr/results}"
SPEC="/workspace/project/training/tao_rtdetr/experiment.yaml"
TRAIN_IMAGE="nvcr.io/nvidia/tao/tao-toolkit:6.0.0-pyt"
DEPLOY_IMAGE="nvcr.io/nvidia/tao/tao-toolkit:6.0.0-deploy"

case "$ACTION" in
  dry-run|train|evaluate|inference|export|gen_trt_engine) ;;
  *) echo "usage: $0 {dry-run|train|evaluate|inference|export|gen_trt_engine}" >&2; exit 2 ;;
esac

if [[ ! -f "$DATASET/tao_rtdetr_safe/annotations/train.json" ]]; then
  echo "Leakage-safe TAO dataset view missing; run prepare_leakage_safe_tao_split.py first" >&2
  exit 2
fi
if ! command -v nvidia-smi >/dev/null 2>&1; then
  echo "NVIDIA GPU runtime is required. This Mac can validate files but cannot run TAO training." >&2
  exit 3
fi

mkdir -p "$RESULTS"
TASK="$ACTION"
IMAGE="$TRAIN_IMAGE"
EXTRA=()
if [[ "$ACTION" == "dry-run" ]]; then
  TASK="train"
  EXTRA=("train.is_dry_run=true")
fi
if [[ "$ACTION" == "gen_trt_engine" ]]; then
  IMAGE="$DEPLOY_IMAGE"
fi

docker run --rm --gpus all \
  -v "$DATASET:/workspace/data:ro" \
  -v "$ROOT:/workspace/project:ro" \
  -v "$RESULTS:/workspace/results" \
  "$IMAGE" rtdetr "$TASK" -e "$SPEC" "${EXTRA[@]}"
