from typing import Any, Optional
from dataclasses import dataclass
import PIL.Image
import PIL.ImageDraw
import io
from .detection import Detection


@dataclass
class Image():
    camera_id: str
    data: Any
    time: float = 0  # World time of recording
    detections: Optional[list[Detection]] = None

    @staticmethod
    def create_placeholder(text: str, time: float = None):
        img = PIL.Image.new('RGB', (260, 200), color=(73, 109, 137))
        d = PIL.ImageDraw.Draw(img)
        d.text((img.width/2-len(text)*3, img.height/2-5), text, fill=(255, 255, 255))
        bytesio = io.BytesIO()
        img.save(bytesio, format='PNG')
        return Image(camera_id='no_cam_id', data=bytesio.getvalue(), time=time or 0)
