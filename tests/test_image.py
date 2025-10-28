import numpy as np
import PIL.Image

from rosys.vision import Image, ImageSize


def test_image_size_tuple():
    size = ImageSize(width=640, height=480)
    assert size.tuple == (640, 480)


def test_image_array_conversion():
    original_array = np.zeros((480, 640, 3), dtype=np.uint8)
    img = Image.from_array(original_array)

    converted_array = img.array
    assert isinstance(converted_array, np.ndarray)
    assert converted_array.shape == (480, 640, 3)
    assert np.allclose(converted_array, original_array, atol=1)


def test_image_pil_conversion():
    original_pil = PIL.Image.new('RGB', (640, 480))
    img = Image.from_pil(original_pil)

    converted_pil = img.to_pil()
    assert isinstance(converted_pil, PIL.Image.Image)
    assert converted_pil.size == (640, 480)
    assert np.allclose(np.array(original_pil), np.array(converted_pil), atol=1)
