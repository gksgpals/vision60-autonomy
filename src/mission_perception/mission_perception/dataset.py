"""Training-dataset contract and leakage-safe validation."""

import hashlib
import json
from pathlib import Path

import cv2


PERCEPTION_CATEGORIES = (
    'person',
    'fire',
    'smoke',
    'gas_cylinder',
    'hazmat_placard',
    'structural_hazard',
)
REQUIRED_SPLITS = ('train', 'val', 'test')


class DatasetValidationError(ValueError):
    """Raised when training data cannot be trusted for an experiment."""


def file_sha256(path):
    """Return a streaming SHA-256 digest."""
    digest = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _load_json(path):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as error:
        raise DatasetValidationError(f'cannot read JSON {path}: {error}') \
            from error


def _validate_categories(categories, location):
    expected = [
        {'id': index, 'name': name}
        for index, name in enumerate(PERCEPTION_CATEGORIES)
    ]
    normalized = [
        {'id': item.get('id'), 'name': item.get('name')}
        for item in categories
    ]
    if normalized != expected:
        raise DatasetValidationError(
            f'{location}: categories must be contiguous and equal to {expected}'
        )


def validate_dataset(dataset_root):
    """Validate files, COCO boxes, source groups, hashes, and split leakage."""
    root = Path(dataset_root)
    manifest_path = root / 'dataset_manifest.json'
    manifest = _load_json(manifest_path)
    if manifest.get('schema_version') != 1:
        raise DatasetValidationError('dataset_manifest schema_version must be 1')
    _validate_categories(manifest.get('categories', []), manifest_path)
    required_categories = manifest.get(
        'required_categories', list(PERCEPTION_CATEGORIES)
    )
    if (
        not required_categories
        or len(required_categories) != len(set(required_categories))
        or any(name not in PERCEPTION_CATEGORIES for name in required_categories)
    ):
        raise DatasetValidationError(
            'required_categories must be a non-empty unique canonical subset'
        )
    splits = manifest.get('splits', {})
    if tuple(splits.keys()) != REQUIRED_SPLITS:
        raise DatasetValidationError(
            f'splits must be ordered exactly as {REQUIRED_SPLITS}'
        )
    sources = manifest.get('sources', [])
    source_ids = set()
    for source in sources:
        source_id = source.get('source_id', '')
        if not source_id or source_id in source_ids:
            raise DatasetValidationError('source_id must be non-empty and unique')
        if not source.get('license') or not source.get('revision'):
            raise DatasetValidationError(
                f'source {source_id} is missing license or revision'
            )
        source_ids.add(source_id)
    if not source_ids:
        raise DatasetValidationError('at least one dataset source is required')

    seen_hashes = {}
    seen_groups = {}
    class_counts = {name: 0 for name in PERCEPTION_CATEGORIES}
    image_counts = {}
    annotation_counts = {}
    for split_name, split in splits.items():
        annotation_path = root / split.get('annotations', '')
        coco = _load_json(annotation_path)
        _validate_categories(coco.get('categories', []), annotation_path)
        images = coco.get('images', [])
        annotations = coco.get('annotations', [])
        image_by_id = {}
        for image in images:
            image_id = image.get('id')
            if image_id in image_by_id:
                raise DatasetValidationError(
                    f'{split_name}: duplicate image id {image_id}'
                )
            source_id = image.get('source_id', '')
            source_group = image.get('source_group', '')
            if source_id not in source_ids or not source_group:
                raise DatasetValidationError(
                    f'{split_name}: image {image_id} has invalid source metadata'
                )
            previous_split = seen_groups.get(source_group)
            if previous_split is not None and previous_split != split_name:
                raise DatasetValidationError(
                    f'source_group leakage: {source_group} is in '
                    f'{previous_split} and {split_name}'
                )
            seen_groups[source_group] = split_name
            image_path = root / image.get('file_name', '')
            if not image_path.is_file():
                raise DatasetValidationError(f'missing image: {image_path}')
            actual_hash = file_sha256(image_path)
            if actual_hash != image.get('sha256'):
                raise DatasetValidationError(
                    f'{split_name}: SHA-256 mismatch for {image_path}'
                )
            previous_hash_split = seen_hashes.get(actual_hash)
            if (
                previous_hash_split is not None
                and previous_hash_split != split_name
            ):
                raise DatasetValidationError(
                    f'image-content leakage between {previous_hash_split} '
                    f'and {split_name}: {image_path}'
                )
            seen_hashes[actual_hash] = split_name
            decoded = cv2.imread(str(image_path))
            if decoded is None:
                raise DatasetValidationError(f'cannot decode image: {image_path}')
            height, width = decoded.shape[:2]
            if width != image.get('width') or height != image.get('height'):
                raise DatasetValidationError(
                    f'{split_name}: dimension mismatch for {image_path}'
                )
            image_by_id[image_id] = image

        annotation_ids = set()
        for annotation in annotations:
            annotation_id = annotation.get('id')
            if annotation_id in annotation_ids:
                raise DatasetValidationError(
                    f'{split_name}: duplicate annotation id {annotation_id}'
                )
            annotation_ids.add(annotation_id)
            image = image_by_id.get(annotation.get('image_id'))
            category_id = annotation.get('category_id')
            if image is None or not isinstance(category_id, int):
                raise DatasetValidationError(
                    f'{split_name}: annotation {annotation_id} has bad references'
                )
            if not 0 <= category_id < len(PERCEPTION_CATEGORIES):
                raise DatasetValidationError(
                    f'{split_name}: invalid category id {category_id}'
                )
            bbox = annotation.get('bbox', [])
            if len(bbox) != 4:
                raise DatasetValidationError(
                    f'{split_name}: annotation {annotation_id} has invalid bbox'
                )
            x_value, y_value, width, height = map(float, bbox)
            if (
                x_value < 0.0 or y_value < 0.0
                or width <= 0.0 or height <= 0.0
                or x_value + width > image['width'] + 1e-6
                or y_value + height > image['height'] + 1e-6
            ):
                raise DatasetValidationError(
                    f'{split_name}: annotation {annotation_id} is outside image'
                )
            expected_area = width * height
            if abs(float(annotation.get('area', -1.0)) - expected_area) > 1e-3:
                raise DatasetValidationError(
                    f'{split_name}: annotation {annotation_id} area mismatch'
                )
            class_counts[PERCEPTION_CATEGORIES[category_id]] += 1
        image_counts[split_name] = len(images)
        annotation_counts[split_name] = len(annotations)

    missing_classes = [
        name for name in required_categories if class_counts[name] == 0
    ]
    if missing_classes:
        raise DatasetValidationError(
            'dataset has no labels for classes: ' + ', '.join(missing_classes)
        )
    return {
        'passed': True,
        'dataset_id': manifest.get('dataset_id', ''),
        'image_counts': image_counts,
        'annotation_counts': annotation_counts,
        'class_counts': class_counts,
        'required_categories': required_categories,
        'source_count': len(source_ids),
        'source_group_count': len(seen_groups),
        'unique_image_hashes': len(seen_hashes),
        'quality': manifest.get('quality', {}),
    }
