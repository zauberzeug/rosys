from collections.abc import Sequence

import cv2
import numpy as np
import pytest

from rosys.geometry import Point, Polygon
from rosys.vision import Image, ImageMasks


def _polygon(*coordinates: tuple[float, float]) -> Polygon:
    return Polygon(outline=[Point(x=x, y=y) for x, y in coordinates])


LEFT_HALF = _polygon((0, 0), (0.5, 0), (0.5, 1), (0, 1))
RIGHT_HALF = _polygon((0.5, 0), (1, 0), (1, 1), (0.5, 1))
FULL_RANGE = _polygon((0, 0), (1, 0), (1, 1), (0, 1))


def _image(camera_id: str = 'cam', *, width: int = 8, height: int = 4) -> Image:
    array = np.random.randint(1, 255, (height, width, 3), dtype=np.uint8)
    return Image.from_array(array, camera_id=camera_id)


def _rasterized(image: Image, polygons: Sequence[Polygon]) -> np.ndarray:
    """Mask an image by rasterizing every polygon over the full image."""
    height, width = image.array.shape[:2]
    kept = np.zeros((height, width), dtype=np.uint8)
    for polygon in polygons:
        contour = np.round([[point.x * width, point.y * height] for point in polygon.outline]).astype(np.int32)
        cv2.fillPoly(kept, [contour], 1)
    return np.where(kept[..., None] == 1, image.array, np.uint8(ImageMasks.FILL_VALUE))


def test_pixels_inside_the_polygons_and_on_their_outline_are_kept_and_the_rest_is_filled():
    masks = ImageMasks({'cam': [LEFT_HALF]})
    image = _image()
    masked = masks.apply(image)
    assert np.array_equal(masked[:, :5], image.array[:, :5])  # the outline runs through column 4
    assert np.all(masked[:, 5:] == ImageMasks.FILL_VALUE)


def test_a_polygon_over_the_full_range_keeps_the_whole_image():
    masks = ImageMasks({'cam': [FULL_RANGE]})
    image = _image()
    assert np.array_equal(masks.apply(image), image.array)


def test_several_polygons_are_combined():
    masks = ImageMasks({'cam': [LEFT_HALF, RIGHT_HALF]})
    image = _image()
    assert np.array_equal(masks.apply(image), image.array)


def test_overlapping_and_nested_polygons_keep_their_union():
    left = _polygon((0, 0), (0.7, 0), (0.7, 1), (0, 1))
    right = _polygon((0.3, 0), (1, 0), (1, 1), (0.3, 1))
    nested = _polygon((0.1, 0.25), (0.4, 0.25), (0.4, 0.75), (0.1, 0.75))
    masks = ImageMasks({'cam': [left, right, nested]})
    image = _image(width=20, height=8)
    assert np.array_equal(masks.apply(image), image.array)


@pytest.mark.parametrize('seed', range(20))
def test_random_polygons_match_a_rasterization_over_the_full_image(seed: int):
    rng = np.random.default_rng(seed)
    coordinates = [0.0, 1.0, *rng.random(6)]
    polygons = [_polygon(*[(rng.choice(coordinates), rng.choice(coordinates)) for _ in range(rng.integers(3, 9))])
                for _ in range(rng.integers(1, 4))]
    masks = ImageMasks({'cam': polygons})
    image = _image(width=int(rng.integers(5, 80)), height=int(rng.integers(5, 60)))
    assert np.array_equal(masks.apply(image), _rasterized(image, polygons))


def test_the_masked_bytes_match_the_masked_array():
    masks = ImageMasks({'cam': [LEFT_HALF]})
    image = _image()
    assert masks.apply_as_bytes(image) == masks.apply(image).tobytes()
    unmasked = _image('other')
    assert masks.apply_as_bytes(unmasked) == unmasked.array.tobytes()


def test_the_image_is_not_modified_in_place():
    masks = ImageMasks({'cam': [LEFT_HALF]})
    image = _image()
    original = image.array.copy()
    masked = masks.apply(image)
    assert masked is not image.array
    assert np.array_equal(image.array, original)


def test_cameras_without_polygons_pass_through_without_a_copy():
    masks = ImageMasks({'cam': [LEFT_HALF]})
    image = _image('other')
    assert masks.apply(image) is image.array
    assert 'other' not in masks
    assert 'cam' in masks


def test_polygons_scale_with_the_image_size():
    masks = ImageMasks({'cam': [LEFT_HALF]})
    for width, height in [(8, 4), (16, 8), (8, 4)]:
        image = _image(width=width, height=height)
        masked = masks.apply(image)
        kept = width // 2 + 1
        assert np.array_equal(masked[:, :kept], image.array[:, :kept])
        assert np.all(masked[:, kept:] == ImageMasks.FILL_VALUE)


def test_setting_new_polygons_replaces_the_old_ones():
    masks = ImageMasks({'cam': [LEFT_HALF]})
    image = _image()
    assert np.all(masks.apply(image)[:, 5:] == ImageMasks.FILL_VALUE)
    masks.set('cam', [RIGHT_HALF])
    assert masks.polygons['cam'] == (RIGHT_HALF,)
    masked = masks.apply(image)
    assert np.all(masked[:, :4] == ImageMasks.FILL_VALUE)
    assert np.array_equal(masked[:, 4:], image.array[:, 4:])


def test_removing_the_polygons_lets_the_camera_pass_through_again():
    masks = ImageMasks({'cam': [LEFT_HALF]})
    masks.remove('cam')
    image = _image()
    assert 'cam' not in masks
    assert masks.apply(image) is image.array


@pytest.mark.parametrize('polygons, message', [
    ([], 'At least one polygon'),
    ([_polygon((0, 0), (1, 1))], 'at least three points'),
    ([_polygon((0, 0), (1.5, 0), (1, 1))], 'outside the normalized range'),
    ([_polygon((0, -0.1), (1, 0), (1, 1))], 'outside the normalized range'),
])
def test_invalid_polygons_are_rejected_when_they_are_set(polygons: list[Polygon], message: str):
    masks = ImageMasks()
    with pytest.raises(ValueError, match=message):
        masks.set('cam', polygons)
    with pytest.raises(ValueError, match=message):
        ImageMasks({'cam': polygons})
    assert 'cam' not in masks
