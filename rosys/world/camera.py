
from collections import deque
from typing import Any
from pydantic import BaseModel, Field
from dataclasses import dataclass
from PIL import Image, ImageDraw
import io

img = Image.new('RGB', (800, 600), color=(73, 109, 137))
d = ImageDraw.Draw(img)
d.text((340, 280), "no image", fill=(255, 255, 255))
placeholder = io.BytesIO()
img.save(placeholder, format='PNG')
placeholder = placeholder.getvalue()


@dataclass
class Frame():
    data: Any
    time: float = 0  # World time of recording


class Camera(BaseModel):
    id: str
    exposure: float = 0
    capture: bool = True
    frames: deque[Frame] = Field(deque([Frame(data=placeholder)], maxlen=10), exclude=True, repr=False)

    @property
    def latest_frame_uri(self):
        return f'camera/{self.id}/{self.frames[-1].time}'
