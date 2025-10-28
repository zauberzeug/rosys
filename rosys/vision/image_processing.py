import io

import cv2
import numpy as np
import PIL.Image

from ..geometry import Rectangle
from .image_rotation import ImageRotation


def encode_image_as_jpeg(image: np.ndarray) -> bytes:
    # TODO: see if we can use this in more places
    return cv2.imencode('.png', image[:, :, ::-1])[1].tobytes()  # NOTE: cv2 expects BGR


def decode_jpeg_image(jpeg_bytes: bytes) -> np.ndarray:
    return np.array(PIL.Image.open(io.BytesIO(jpeg_bytes)))


def process_jpeg_image(data: bytes, rotation: ImageRotation, crop: Rectangle | None = None) -> np.ndarray:
    """Rotate and crop a JPEG image and return it as a NumPy pixel array"""
    array = decode_jpeg_image(data)
    return process_ndarray_image(array, rotation, crop)


def process_ndarray_image(image: np.ndarray, rotation: ImageRotation, crop: Rectangle | None = None) -> np.ndarray:
    """Rotate and crop a NumPy image."""
    if crop is not None:
        image = image[int(crop.y):int(crop.y+crop.height), int(crop.x):int(crop.x+crop.width)]
    if rotation == ImageRotation.LEFT:
        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    elif rotation == ImageRotation.RIGHT:
        image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    elif rotation == ImageRotation.UPSIDE_DOWN:
        image = cv2.rotate(image, cv2.ROTATE_180)
    return image


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
