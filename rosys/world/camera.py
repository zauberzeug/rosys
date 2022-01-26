from pydantic import BaseModel, Field
from .frame import Frame


class Camera(BaseModel):
    id: str
    capture: bool = True
    detect: bool = False
    frames: list[Frame] = Field([Frame.create_placeholder('no image')], exclude=True)

    @property
    def latest_frame_uri(self):
        return f'camera/{self.id}/{self.frames[-1].time}'
