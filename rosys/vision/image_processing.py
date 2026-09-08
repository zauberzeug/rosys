from __future__ import annotations

import io
import logging
from io import BytesIO
from typing import TYPE_CHECKING

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

DEFAULT_JPEG_QUALITY = 90
"""JPEG quality (1-100) used wherever no quality is requested explicitly."""


def validate_jpeg_quality(quality: int) -> None:
    """Ensure a JPEG quality is within the range both encoders accept.

    TurboJPEG only reports an unspecific ``Invalid argument`` for out-of-range values,
    and the failure would surface deep inside an encode - possibly per frame.

    :param quality: the JPEG quality to check.
    :raises ValueError: if the quality is not between 1 and 100.
    """
    if not 1 <= quality <= 100:
        raise ValueError(f'JPEG quality must be between 1 and 100. Got "{quality}"')


def encode_image_as_jpeg(image: np.ndarray, quality: int = DEFAULT_JPEG_QUALITY) -> bytes:
    """Encode image as JPEG using TurboJPEG if available, otherwise PIL.

    :param image: the pixel array to encode.
    :param quality: JPEG quality from 1 (smallest file) to 100 (best quality).
    :return: the encoded JPEG bytes.
    :raises ValueError: if the quality is not between 1 and 100.
    """
    validate_jpeg_quality(quality)

    if TURBO_JPEG is not None:
        return TURBO_JPEG.encode(image, quality=quality)

    pil_image = PIL.Image.fromarray(image.astype(np.uint8))

    buffer = BytesIO()
    pil_image.save(buffer, format='JPEG', quality=quality)
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
        h, w = image.shape[:2]
        x1 = max(0, int(crop.x))
        y1 = max(0, int(crop.y))
        x2 = min(w, int(crop.x + crop.width))
        y2 = min(h, int(crop.y + crop.height))
        image = image[y1:y2, x1:x2]
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
