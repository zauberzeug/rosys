from collections.abc import Sequence

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


def _pixel_centers(polygons: Sequence[Polygon], width: int, height: int) -> tuple[np.ndarray, np.ndarray]:
    """Return per pixel whether its center lies inside a polygon and its distance to the nearest outline in pixels."""
    x, y = np.meshgrid(np.arange(width) + 0.5, np.arange(height) + 0.5)
    inside = np.zeros((height, width), dtype=bool)
    distance = np.full((height, width), np.inf)
    for polygon in polygons:
        corners = [(point.x * width, point.y * height) for point in polygon.outline]
        crossings = np.zeros((height, width), dtype=bool)
        for (x0, y0), (x1, y1) in zip(corners, corners[1:] + corners[:1], strict=True):
            with np.errstate(divide='ignore', invalid='ignore'):
                crossings ^= ((y0 > y) != (y1 > y)) & (x < x0 + (y - y0) * (x1 - x0) / (y1 - y0))
            dx, dy = x1 - x0, y1 - y0
            t = np.clip(((x - x0) * dx + (y - y0) * dy) / max(dx * dx + dy * dy, 1e-12), 0, 1)
            distance = np.minimum(distance, np.hypot(x - x0 - t * dx, y - y0 - t * dy))
        inside |= crossings
    return inside, distance


def test_pixels_inside_the_polygons_and_on_their_outline_are_filled_and_the_rest_is_kept():
    masks = ImageMasks({'cam': [LEFT_HALF]})
    image = _image()
    masked = masks.apply(image)
    assert np.all(masked[:, :5] == ImageMasks.FILL_VALUE)  # the outline runs through column 4
    assert np.array_equal(masked[:, 5:], image.array[:, 5:])


def test_a_polygon_over_the_full_range_fills_the_whole_image():
    masks = ImageMasks({'cam': [FULL_RANGE]})
    assert np.all(masks.apply(_image()) == ImageMasks.FILL_VALUE)


def test_several_polygons_are_combined():
    masks = ImageMasks({'cam': [LEFT_HALF, RIGHT_HALF]})
    assert np.all(masks.apply(_image()) == ImageMasks.FILL_VALUE)


def test_overlapping_and_nested_polygons_fill_their_union():
    left = _polygon((0, 0), (0.7, 0), (0.7, 1), (0, 1))
    right = _polygon((0.3, 0), (1, 0), (1, 1), (0.3, 1))
    nested = _polygon((0.1, 0.25), (0.4, 0.25), (0.4, 0.75), (0.1, 0.75))
    masks = ImageMasks({'cam': [left, right, nested]})
    assert np.all(masks.apply(_image(width=20, height=8)) == ImageMasks.FILL_VALUE)


@pytest.mark.parametrize('seed', range(20))
def test_random_polygons_are_filled_to_within_a_pixel_of_their_outlines(seed: int):
    rng = np.random.default_rng(seed)
    coordinates = [0.0, 1.0, *rng.random(6)]
    polygons = [_polygon(*[(rng.choice(coordinates), rng.choice(coordinates)) for _ in range(rng.integers(3, 9))])
                for _ in range(rng.integers(1, 4))]
    width, height = int(rng.integers(5, 80)), int(rng.integers(5, 60))
    image = Image.from_array(np.zeros((height, width, 3), dtype=np.uint8), camera_id='cam')
    masked = ImageMasks({'cam': polygons}).apply(image)
    filled = np.all(masked == ImageMasks.FILL_VALUE, axis=2)
    assert np.all(filled | np.all(masked == 0, axis=2))
    inside, distance = _pixel_centers(polygons, width, height)
    assert np.all(filled[inside & (distance > 0.5)])
    assert np.all(inside[filled] | (distance[filled] <= 1.5))


@pytest.mark.parametrize('polygon, filled', [
    (_polygon((0, 0), (0.05, 0), (0.05, 1), (0, 1)), np.s_[:, :1]),
    (_polygon((0.95, 0), (1, 0), (1, 1), (0.95, 1)), np.s_[:, 7:]),
    (_polygon((0, 0), (1, 0), (1, 0.05), (0, 0.05)), np.s_[:1]),
    (_polygon((0, 0.95), (1, 0.95), (1, 1), (0, 1)), np.s_[3:]),
])
def test_thin_polygons_along_the_image_borders_fill_the_outermost_pixels(polygon: Polygon,
                                                                         filled: slice | tuple[slice, slice]):
    image = _image()
    expected = image.array.copy()
    expected[filled] = ImageMasks.FILL_VALUE
    assert np.array_equal(ImageMasks({'cam': [polygon]}).apply(image), expected)


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
    for width, height in [(8, 4), (16, 8)]:
        image = _image(width=width, height=height)
        masked = masks.apply(image)
        filled = width // 2 + 1
        assert np.all(masked[:, :filled] == ImageMasks.FILL_VALUE)
        assert np.array_equal(masked[:, filled:], image.array[:, filled:])


def test_setting_new_polygons_replaces_the_old_ones():
    masks = ImageMasks({'cam': [LEFT_HALF]})
    image = _image()
    assert np.all(masks.apply(image)[:, :5] == ImageMasks.FILL_VALUE)
    masks.set('cam', [RIGHT_HALF])
    assert masks.polygons['cam'] == (RIGHT_HALF,)
    masked = masks.apply(image)
    assert np.array_equal(masked[:, :4], image.array[:, :4])
    assert np.all(masked[:, 4:] == ImageMasks.FILL_VALUE)


def test_polygons_can_be_given_as_any_iterable():
    masks = ImageMasks()
    masks.set('cam', (polygon for polygon in [LEFT_HALF, RIGHT_HALF]))
    assert masks.polygons['cam'] == (LEFT_HALF, RIGHT_HALF)


def test_setting_no_polygons_lets_the_camera_pass_through():
    masks = ImageMasks({'cam': [LEFT_HALF], 'other': []})
    masks.set('cam', [])
    image = _image()
    assert 'cam' not in masks
    assert 'other' not in masks
    assert masks.apply(image) is image.array


def test_removing_the_polygons_lets_the_camera_pass_through_again():
    masks = ImageMasks({'cam': [LEFT_HALF]})
    masks.remove('cam')
    image = _image()
    assert 'cam' not in masks
    assert masks.apply(image) is image.array


@pytest.mark.parametrize('polygons, message', [
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
