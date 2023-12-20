import time

import cv2
import numpy as np
import PIL

from ... import rosys
from ...geometry import Point
from ..image import ImageSize


class SimulatedDevice:

    def __init__(self,
                 id: str,  # pylint: disable=redefined-builtin
                 size: ImageSize,
                 color: str = '#ffffff') -> None:
        self.id = id
        self.size = size
        self.color = color
        self.creation_time: float = rosys.time()

    def create_image_data(self) -> bytes:
        img = PIL.Image.new('RGB', size=(self.size.width, self.size.height), color=self.color)
        d = PIL.ImageDraw.Draw(img)
        text = f'{self.id}: {time.time():.2f}'
        position = floating_text_position(self.size.width, self.size.height)

        d.text((position.x, position.y), text, fill=(0, 0, 0))
        d.text((position.x + 1, position.y + 1), text, fill=(255, 255, 255))

        _, encoded_image = cv2.imencode('.jpg', np.array(img)[:, :, ::-1])  # NOTE: cv2 expects BGR
        return encoded_image.tobytes()


def floating_text_position(box_width: int, box_height: int, speed: float = 100, angle: float = np.deg2rad(45)) -> Point:
    """Calculate the position of the text that floats around the image."""
    t = time.time()
    return Point(x=(speed * t * np.cos(angle)) % box_width,
                 y=(speed * t * np.sin(angle)) % box_height)
