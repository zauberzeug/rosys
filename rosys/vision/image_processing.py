from __future__ import annotations

import io
import logging
from io import BytesIO
from typing import TYPE_CHECKING, Any

import cv2
import numpy as np
import PIL.Image

from ..geometry import Rectangle
from .image_rotation import ImageRotation
from .turbojpeg_wrapper import TURBO_JPEG

if TYPE_CHECKING:
    from .image import ImageArray

if not TURBO_JPEG:
    logging.getLogger('rosys').warning('TurboJPEG is not available. Using PIL for JPEG decoding and encoding.')


def encode_image_as_jpeg(image: np.ndarray, compression_level: int = 90) -> bytes:
    """Encode image as JPEG using TurboJPEG if available, otherwise PIL."""
    if TURBO_JPEG is not None:
        return TURBO_JPEG.encode(image, quality=compression_level)

    pil_image = PIL.Image.fromarray(image.astype(np.uint8))

    buffer = BytesIO()
    save_kwargs: dict[str, Any] = {}
    if compression_level is not None:
        save_kwargs['quality'] = compression_level

    pil_image.save(buffer, format='JPEG', **save_kwargs)
    return buffer.getvalue()


def decode_jpeg_image(jpeg_bytes: bytes) -> ImageArray | None:
    """Decode JPEG bytes to NumPy array using TurboJPEG if available, otherwise PIL. Returns None if decoding failed."""
    try:
        if TURBO_JPEG is not None:
            return TURBO_JPEG.decode(jpeg_bytes)
        return np.array(PIL.Image.open(io.BytesIO(jpeg_bytes)))
    except (PIL.UnidentifiedImageError, OSError) as e:
        logging.warning('Failed to decode JPEG image %s', e)
        return None


def process_jpeg_image(data: bytes, rotation: ImageRotation, crop: Rectangle | None = None) -> np.ndarray | None:
    """Rotate and crop a JPEG image and return it as a NumPy pixel array. Returns None if decoding failed."""
    array = decode_jpeg_image(data)
    if array is None:
        return None
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
