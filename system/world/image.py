from pydantic import BaseModel


class Image(BaseModel):

    id: str
    time: float
    mac: str
