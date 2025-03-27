import numpy as np
import PIL.Image
import pytest

from rosys.vision.image import Image, ImageSize


def sample_image_data():
    return PIL.Image.new('RGB', (100, 100), color='red')


def test_image_size_tuple():
    size = ImageSize(width=1920, height=1080)
    assert size.tuple == (1920, 1080)


def test_image_array_conversion():
    original_array = np.array(sample_image_data())
    img = Image.from_array(original_array)

    converted_array = img.to_array()
    assert isinstance(converted_array, np.ndarray)
    assert converted_array.shape == (100, 100, 3)
    assert np.allclose(converted_array, original_array, atol=1)


def test_image_pil_conversion():
    original_pil = sample_image_data()
    img = Image.from_pil(original_pil)

    converted_pil = img.to_pil()
    assert isinstance(converted_pil, PIL.Image.Image)
    assert converted_pil.size == (100, 100)

    assert np.allclose(np.array(original_pil), np.array(converted_pil), atol=1)


def test_image_without_data():
    img = Image(camera_id='test_cam', time=123.45, size=ImageSize(width=100, height=100))
    with pytest.raises(ValueError, match='Image data is None'):
        img.to_array()
