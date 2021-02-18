from pydantic.main import BaseModel
from pydantic.types import PositiveFloat
from requests import Session
from typing import Generator
from urllib.parse import urljoin
from fastapi.encoders import jsonable_encoder
from robot import Pose
from easy_vector import Vector as V
from retry import retry


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

    @retry(AssertionError, tries=3, delay=0.1)
    def assert_pose(self, x: int, y: int, orientation: int = 0):
        assert self.pose is not None
        assert self.pose.location is not None
        assert self.pose.location == V(x, y)
        assert self.pose.orientation == orientation
