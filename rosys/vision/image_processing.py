import io
import logging
from typing import Optional

import cv2
import imgsize
import numpy as np

from ..geometry import Rectangle
from .image import ImageSize
from .image_rotation import ImageRotation


class PeekableBytesIO(io.BytesIO):

    def peek(self, n=-1):
        position = self.tell()
        data = self.read(n)
        self.seek(position)
        return data


def get_image_size_from_bytes(image: bytes) -> ImageSize:
    try:
        with PeekableBytesIO(image) as f:
            width, height = imgsize.get_size(f)
    except imgsize.UnknownSize as e:
        raise ValueError('Could not determine image size') from e
    return ImageSize(width=width, height=height)


def encode_image_as_jpeg(image: np.ndarray) -> bytes:
    return cv2.imencode('.jpg', image)[1].tobytes()


def process_jpeg_image(data: bytes, rotation: ImageRotation, crop: Optional[Rectangle] = None) -> bytes | None:
    """Rotate and crop a JPEG image."""
    if crop is None and rotation == ImageRotation.NONE:
        return data
    array = np.frombuffer(data, dtype=np.uint8)
    decoded = cv2.imdecode(array, cv2.IMREAD_COLOR)
    if decoded is None:
        logging.warning('could not decode image buffer')
        return None
    return process_ndarray_image(decoded, rotation, crop)


def process_ndarray_image(image: np.ndarray, rotation: ImageRotation, crop: Optional[Rectangle] = None) -> bytes:
    """Rotate and crop a NumPy image and encode it as JPEG."""
    if crop is not None:
        image = image[int(crop.y):int(crop.y+crop.height), int(crop.x):int(crop.x+crop.width)]
    if rotation == ImageRotation.LEFT:
        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    elif rotation == ImageRotation.RIGHT:
        image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    elif rotation == ImageRotation.UPSIDE_DOWN:
        image = cv2.rotate(image, cv2.ROTATE_180)
    return encode_image_as_jpeg(image)


def remove_exif(image_data: bytes) -> bytes:  # written by ChatGPT
    pos = 2  # Skip SOI marker
    while pos < len(image_data):
        if image_data[pos] == 0xFF:
            if image_data[pos + 1] == 0xE1:  # APP1 marker (EXIF)
                length = image_data[pos + 2] << 8 | image_data[pos + 3]
                image_data = image_data[:pos] + image_data[pos + length + 2:]
                continue  # Check for multiple EXIF segments
            elif image_data[pos + 1] == 0xD9:  # EOI marker
                break  # End of image
            else:
                length = image_data[pos + 2] << 8 | image_data[pos + 3]
                pos += length + 2  # Skip to next marker
        else:
            pos += 1  # Increment position if not a marker
    return image_data
