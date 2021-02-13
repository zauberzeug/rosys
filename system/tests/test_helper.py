from pydantic.main import BaseModel
from requests import Session
from typing import Generator
from urllib.parse import urljoin
from fastapi.encoders import jsonable_encoder
from robot import Pose


class LiveServerSession(Session):
    """https://stackoverflow.com/a/51026159/364388"""

    def __init__(self, *args, **kwargs):
        super(LiveServerSession, self).__init__(*args, **kwargs)
        self.prefix_url = 'http://localhost:80/api'

    def request(self, method, url, *args, **kwargs):
        url = urljoin(self.prefix_url, url)
        return super(LiveServerSession, self).request(method, url, *args, **kwargs)


def assert_properties(obj, **kwargs):
    for key, value in kwargs.items():
        assert getattr(obj, key) == value


class Robot(BaseModel):
    pose: Pose = None
