import io
import logging

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


def process_jpeg_image(data: bytes, rotation: ImageRotation, crop: Rectangle | None = None) -> bytes | None:
    """Rotate and crop a JPEG image."""
    if crop is None and rotation == ImageRotation.NONE:
        return data
    array = np.frombuffer(data, dtype=np.uint8)
    decoded = cv2.imdecode(array, cv2.IMREAD_COLOR)
    if decoded is None:
        logging.warning('could not decode image buffer')
        return None
    return process_ndarray_image(decoded, rotation, crop)


def process_ndarray_image(image: np.ndarray, rotation: ImageRotation, crop: Rectangle | None = None) -> bytes:
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


def remove_exif(image_data: bytes | bytearray) -> bytes:
    EXIF_MARKER = b'\xFF\xE1'

    pos = 2  # Skip SOI marker
    image_data = bytearray(image_data)
    while pos < len(image_data):
        match_start = image_data.find(EXIF_MARKER, pos)
        if match_start == -1:
            break
        match_end = match_start + 2
        length = int.from_bytes(image_data[match_end:match_end+2], byteorder='big')
        del image_data[match_start:match_end+length]  # Remove EXIF segment
        pos = match_end

    return bytes(image_data)
