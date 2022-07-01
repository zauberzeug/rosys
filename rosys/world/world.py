from pydantic import BaseModel

from .upload import Upload


class World(BaseModel):
    upload: Upload = Upload()
