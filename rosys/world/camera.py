
from typing import Any, List
from pydantic import BaseModel, Field
from dataclasses import dataclass
from PIL import Image, ImageDraw
import io


@dataclass
class Frame():
    data: Any
    time: float = 0  # World time of recording

    @staticmethod
    def create_placeholder(text: str, time: float = None):
        img = Image.new('RGB', (260, 200), color=(73, 109, 137))
        d = ImageDraw.Draw(img)
        d.text((img.width/2-len(text)*3, img.height/2-5), text, fill=(255, 255, 255))
        bytesio = io.BytesIO()
        img.save(bytesio, format='PNG')
        return Frame(data=bytesio.getvalue(), time=time or 0)


no_img_placeholder = Frame.create_placeholder('no image')


class Camera(BaseModel):
    id: str
    exposure: float = 0
    capture: bool = True
    frames: List[Frame] = Field([no_img_placeholder], exclude=True)

    @property
    def latest_frame_uri(self):
        return f'camera/{self.id}/{self.frames[-1].time}'
