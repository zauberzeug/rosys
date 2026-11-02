import numpy as np
import pytest

from rosys.geometry import Point, Polygon
from rosys.vision import Image, ImageMasks, ImageSize


def _polygon(*coordinates: tuple[float, float]) -> Polygon:
    return Polygon(outline=[Point(x=x, y=y) for x, y in coordinates])


LEFT_HALF = _polygon((0, 0), (0.5, 0), (0.5, 1), (0, 1))
RIGHT_HALF = _polygon((0.5, 0), (1, 0), (1, 1), (0.5, 1))
# pixels touched by a polygon edge are kept, so a column right at the boundary is excluded from the assertions


def _image(camera_id: str = 'cam', *, width: int = 8, height: int = 4) -> Image:
    array = np.random.randint(1, 255, (height, width, 3), dtype=np.uint8)
    return Image.from_array(array, camera_id=camera_id)


def test_pixels_inside_the_polygons_are_kept_and_the_rest_is_filled():
    masks = ImageMasks({'cam': [LEFT_HALF]})
    image = _image()
    masked = masks.apply(image)
    assert np.array_equal(masked[:, :4], image.array[:, :4])
    assert np.all(masked[:, 5:] == ImageMasks.FILL_VALUE)


def test_several_polygons_are_combined():
    masks = ImageMasks({'cam': [LEFT_HALF, RIGHT_HALF]})
    image = _image()
    assert np.array_equal(masks.apply(image), image.array)


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
    small = masks.apply(_image(width=8, height=4))
    large = masks.apply(_image(width=16, height=8))
    assert np.all(small[:, 5:] == ImageMasks.FILL_VALUE) and not np.any(small[:, :4] == 0)
    assert np.all(large[:, 9:] == ImageMasks.FILL_VALUE) and not np.any(large[:, :8] == 0)


def test_the_raster_is_cached_per_camera_for_the_last_image_size(monkeypatch: pytest.MonkeyPatch):
    masks = ImageMasks({'cam': [LEFT_HALF]})
    calls: list[tuple[int, int]] = []
    original_raster = masks._raster  # pylint: disable=protected-access

    def counting_raster(camera_id: str, size: ImageSize):
        cached = masks._rasters.get(camera_id)  # pylint: disable=protected-access
        if cached is None or (cached[0], cached[1]) != size.tuple:
            calls.append(size.tuple)
        return original_raster(camera_id, size)
    monkeypatch.setattr(masks, '_raster', counting_raster)

    masks.apply(_image(width=8, height=4))
    masks.apply(_image(width=8, height=4))
    assert calls == [(8, 4)]
    masks.apply(_image(width=16, height=8))
    masks.apply(_image(width=8, height=4))
    assert calls == [(8, 4), (16, 8), (8, 4)]


def test_setting_new_polygons_replaces_the_old_ones_and_their_raster():
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


def test_covers_reports_whether_a_pixel_lies_in_the_kept_region():
    masks = ImageMasks({'cam': [LEFT_HALF]})
    size = ImageSize(width=8, height=4)
    assert masks.covers('cam', Point(x=1, y=1), size)
    assert not masks.covers('cam', Point(x=6, y=1), size)
    assert masks.covers('other', Point(x=6, y=1), size)


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
