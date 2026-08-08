# Vision60 mission-perception training contract

## Decision

Use two stages rather than calling every detected person a rescued victim.

1. NVIDIA PeopleNet AMR provides the initial `person` candidate on Jetson.
2. A custom TensorRT detector adds `fire`, `smoke`, `gas_cylinder`,
   `hazmat_placard`, and `structural_hazard`.
3. Camera boxes are fused with Ouster points by `mission_perception`.
4. Every result is published as a candidate. A person becomes a confirmed
   victim only after operator or additional sensor verification.

Isaac ROS Object Detection `release-3.2` is pinned as the deployment candidate
because that release supports Jetson Orin, JetPack 6.1/6.2, and ROS 2 Humble.
PeopleNet has a robot-height training subset and deployable TensorRT model.
D-Fire is the initial fire/smoke source. Its collection is declared CC0, but
the upstream authors state that they do not own every source image, so source
rights must remain in the dataset manifest and must be reviewed before public
redistribution.

## Canonical classes

The class IDs are fixed and contiguous for TAO/COCO conversion.

| ID | Class | Meaning |
|---:|---|---|
| 0 | `person` | Person candidate; never automatic victim confirmation |
| 1 | `fire` | Visible flame candidate |
| 2 | `smoke` | Visible smoke candidate |
| 3 | `gas_cylinder` | Cylinder or pressurized-container candidate |
| 4 | `hazmat_placard` | Hazard label or placard candidate |
| 5 | `structural_hazard` | Collapse or falling-object hazard candidate |

## Required dataset structure

```text
dataset_root/
├── dataset_manifest.json
├── images/
└── annotations/
    ├── train.json
    ├── val.json
    └── test.json
```

Annotations use COCO bounding boxes. Every image also requires `sha256`,
`source_id`, and `source_group`. A source group is one video, simulation run,
or continuous capture session. Groups and identical image hashes must never
cross train, validation, and test splits.

Run the validator before training:

```bash
ros2 run mission_perception validate_perception_dataset \
  --dataset /data/vision60_perception \
  --report /data/vision60_perception/validation_report.json
```

Convert the official pre-split D-Fire tree (`train/val/test`, each containing
`images/` and `labels/`) before merging it with other sources:

```bash
ros2 run mission_perception import_dfire_dataset \
  --source /data/raw/dfire \
  --output /data/shards/dfire \
  --revision 4bf9c31b18fadcd44d5f0b6d66f82bc56fa5e328 \
  --materialization hardlink \
  --box-policy strict \
  --invalid-box-policy reject
```

The importer maps D-Fire `0=smoke`, `1=fire` into canonical IDs, copies the
images, converts normalized YOLO boxes to COCO, records hashes and source
groups, and runs the validator. Empty label files remain valid negative
samples. Missing labels, unknown classes, invalid boxes, duplicates across
splits, and damaged images fail the import. The D-Fire shard requires only
`fire` and `smoke`; the final merged training dataset still requires all six
canonical classes.

Use `hardlink` only when source and output are on the same filesystem. It
avoids duplicating several gigabytes of images while keeping the converted
dataset valid even if the original directory entry is later removed. For a
repacked distribution, also pass `--distribution-url` and
`--distribution-revision` so the exact download remains traceable.

`strict` rejects every out-of-frame label. If a reviewed distribution contains
boxes that intentionally extend beyond the image, use `--box-policy clip`.
The importer clips them to the visible image and records the repaired count,
maximum overflow, and minimum retained-area ratio in the manifest and report;
it never repairs a box that does not intersect the image.

`reject` also fails on zero-area or non-finite boxes. After a documented data
audit, `--invalid-box-policy drop` may remove only those unusable geometries
while retaining their images. The report records every dropped box by reason;
malformed rows and unknown classes always fail.

## Data plan

- `person`: start with PeopleNet without copying its proprietary training data;
  validate using robot-height staged and synthetic images.
- `fire`, `smoke`: D-Fire 21,527-image shard import and exact-duplicate validation
  are complete; next add indoor, low-light, rubble, fog, steam, and dust hard
  negatives after near-duplicate review.
- remaining hazards: generate digital-twin images, use open-vocabulary
  auto-labeling only as a draft, and require human box review.
- school reopening: capture the actual camera height, lens, lighting, motion
  blur, and OS1 occlusion conditions. Keep this field test split untouched.

## Prepared TAO RT-DETR pipeline

The fire/smoke path now uses NVIDIA TAO 6.0 RT-DETR with a ResNet-18
backbone. This keeps training, evaluation, ONNX export, TensorRT generation,
and Isaac ROS deployment inside the same NVIDIA-supported toolchain.

The upstream split must not be trained directly. The perceptual audit found
many nearly identical sequence frames across train, validation, and test.
Create a scene-cluster-safe two-class view after the audit. Images are
hardlinked, so the 21,527 files do not consume another dataset-sized copy.

```bash
python3 scripts/prepare_leakage_safe_tao_split.py \
  --source datasets.nosync/dfire/coco \
  --audit-report artifacts/dfire_near_duplicate_audit/report.json \
  --output datasets.nosync/dfire/coco/tao_rtdetr_safe
python3 scripts/validate_tao_rtdetr_bundle.py \
  --dataset datasets.nosync/dfire/coco/tao_rtdetr_safe \
  --spec training/tao_rtdetr/experiment.yaml \
  --report artifacts/tao_rtdetr_bundle_validation.json
```

On an NVIDIA GPU host, use the pinned TAO 6.0 training container. TensorRT
engine generation automatically switches to the matching `6.0.0-deploy`
container because NVIDIA separates training and deployment runtimes:

```bash
scripts/run_tao_rtdetr.sh dry-run
scripts/run_tao_rtdetr.sh train
scripts/run_tao_rtdetr.sh evaluate
scripts/run_tao_rtdetr.sh export
scripts/run_tao_rtdetr.sh gen_trt_engine
```

The safe split contains 14,965 train, 3,300 validation, and 3,262 test images.
All PHash+DHash scene components are assigned wholly to one split. The current
Mac completed schema, image-path, class-ID, box, and deployment configuration
validation. It did not claim GPU training or model accuracy; those require an
NVIDIA GPU and are deliberately reported as not executed.

## Isaac ROS fusion contract

Isaac ROS RT-DETR publishes `vision_msgs/Detection2DArray` on
`/detections_output`. Start the project adapter after the detector:

```bash
ros2 launch vision60_bringup mission_perception_rtdetr_fusion.launch.py
```

Class `0` maps to `fire_candidate` and class `1` to `smoke_candidate`.
Detections older than 0.25 seconds, below 0.35 confidence, outside the image,
or with invalid dimensions are rejected. Valid boxes are associated with
Ouster points and transformed into the map frame. They remain candidates and
still require operator or multi-sensor confirmation.

## Near-duplicate audit

`idealo/imagededup` revision
`f0534a6ec10c02379c627696fbd486841068631c` supplies PHash generation and
BK-tree search. Optional CNN, plotting, and wavelet dependencies were changed
to lazy imports so the auditable hash-only path does not install PyTorch.

```bash
python3 scripts/audit_perception_near_duplicates.py \
  --dataset datasets.nosync/dfire/coco \
  --imagededup-source external_src/imagededup \
  --threshold 6 \
  --report artifacts/dfire_near_duplicate_audit/report.json \
  --contact-sheet artifacts/dfire_near_duplicate_audit/candidates.png
```

The broad PHash audit found 220,886 cross-split candidate pairs. Requiring
both PHash and DHash still left 130,787 cross-split pairs in 608 connected
scene components. These components drive the safe re-split. Hash candidates
remain review signals rather than proof of identity, and the audit never
deletes or moves source images automatically.

## Provisional acceptance gates

These are targets, not achieved results.

- In-domain person recall at IoU 0.5: at least 0.90.
- Fire/smoke recall at IoU 0.5: at least 0.85.
- Each production class precision: at least 0.80.
- TensorRT end-to-end perception latency: at most 100 ms at the selected input.
- 3D localization median error: at most 0.35 m; 95th percentile: at most 0.75 m.
- No test image, video, scene, or duplicate content may appear in training.

Report per-class precision, recall, AP50, AP50:95, distance-binned recall,
lighting/occlusion subsets, false alarms per mission minute, 3D position error,
and latency. Do not use only one aggregate mAP value.
