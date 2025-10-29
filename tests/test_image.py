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


def test_image_placeholder():
    img = Image.create_placeholder('hello')

    assert img.size.tuple == Image.DEFAULT_PLACEHOLDER_SIZE


def test_image_from_jpeg_bytes():
    # Smallest valid jpeg: https://github.com/mathiasbynens/small/blob/master/jpeg.jpg
    jpeg_bytes = b'\xff\xd8\xff\xdb\x00C\x00\x03\x02\x02\x02\x02\x02\x03\x02\x02\x02\x03\x03\x03\x03\x04\x06\x04\x04\x04\x04\x04\x08\x06\x06\x05\x06\t\x08\n\n\t\x08\t\t\n\x0c\x0f\x0c\n\x0b\x0e\x0b\t\t\r\x11\r\x0e\x0f\x10\x10\x11\x10\n\x0c\x12\x13\x12\x10\x13\x0f\x10\x10\x10\xff\xc9\x00\x0b\x08\x00\x01\x00\x01\x01\x01\x11\x00\xff\xcc\x00\x06\x00\x10\x10\x05\xff\xda\x00\x08\x01\x01\x00\x00?\x00\xd2\xcf \xff\xd9'
    img = Image.from_jpeg_bytes(jpeg_bytes)
    assert img is not None
    assert img.array == np.array([[190]], dtype=np.uint8)


def test_image_from_jpeg_bytes_invalid():
    not_a_jpg = b'2737'
    truncated = b'\xff\xd8\xff\xdb\x00C\x00'
    invalid_byte = b'\xff\xd8\xff\xdb\x00C\x00\x03\x02\x02\x02\x02\x02\x03\x02\x02\x02\x03\x03\x03\x03\x04\x06\x04\x04\x04\x04\x04\x08\x06\x06\x05\x06\t\x08\n\n\t\x08\t\t\n\x0c\x0f\x0c\n\x0b\x0e\x0b\t\t\r\x11\r\x0e\x0f\x10\x10\x11\x10\n\x0c\x12\x13\x12\x10\x13\x0f\x10\x10\x10\xff\xc9\x00\x0b\x08\x00\x01\x00\x01\x01\x01\x11\x00\xff\xcc\x00\x06\x00\x10\x10\x05\xff\xda\x00\x08\x01\x01\x00\x00?\x00\xd2\xcf \xff\xd8'

    for jpeg_bytes in [not_a_jpg, truncated, invalid_byte]:
        img = Image.from_jpeg_bytes(jpeg_bytes)
        assert img is None
