import io
from typing import Any, Optional

import cv2
import numpy as np
import PIL

from ..geometry import Rectangle
from .image_rotation import ImageRotation


def process_jpeg_image(data: bytes, rotation: ImageRotation, crop: Optional[Rectangle] = None) -> bytes:
    image = PIL.Image.open(io.BytesIO(data))
    if crop is not None:
        image = image.crop((int(crop.x), int(crop.y), int(crop.x + crop.width), int(crop.y + crop.height)))
    image = image.rotate(int(rotation), expand=True)  # NOTE: PIL handles rotation with 90 degree steps efficiently
    img_byte_arr = io.BytesIO()
    image.save(img_byte_arr, format='JPEG')
    return img_byte_arr.getvalue()


def process_ndarray_image(image: np.ndarray, rotation: ImageRotation, crop: Optional[Rectangle] = None) -> bytes:
    if crop is not None:
        image = image[int(crop.y):int(crop.y+crop.height), int(crop.x):int(crop.x+crop.width)]
    if rotation == ImageRotation.LEFT:
        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
    elif rotation == ImageRotation.RIGHT:
        image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
    elif rotation == ImageRotation.UPSIDE_DOWN:
        image = cv2.rotate(image, cv2.ROTATE_180)
    return cv2.imencode('.jpg', image)[1].tobytes()


def to_bytes(image: Any) -> bytes:
    return image[0].tobytes()
