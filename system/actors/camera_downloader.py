import asyncio
from typing import List
import pycurl
import io
from pydantic.main import BaseModel
import simplejpeg
from icecream import ic
from actors.actor import Actor
from world.world import World
import helpers


class Img(BaseModel):

    data: bytes
    mac: str


class CameraDownloader(Actor):

    interval: float = 0

    def __init__(self):

        self.images: List[Img] = []

    async def step(self, world: World):

        timeout = 1

        helpers.measure(reset=True)
        for camera in world.cameras.values():
            url = f'http://{camera.network.ip}:8080/image?maxtime={timeout}&killtime=2.5'
            loop = asyncio.get_event_loop()
            try:
                helpers.measure()
                header, content = await loop.run_in_executor(None, self.get, url, timeout)
                helpers.measure()
            except pycurl.error:
                ic('img download error')
                continue
            jpeg_header = simplejpeg.decode_jpeg_header(content)
            helpers.measure()
            # ic(header)
            # ic(jpeg_header)

            self.images.append(Img(data=content, mac=header['mac']))

    def get(self, url, timeout=1):

        content = io.BytesIO()
        headers = io.BytesIO()

        connection = pycurl.Curl()
        connection.setopt(pycurl.TIMEOUT, timeout)
        connection.setopt(pycurl.URL, url)
        connection.setopt(pycurl.HTTPGET, 1)
        connection.setopt(pycurl.WRITEFUNCTION, content.write)
        connection.setopt(pycurl.HEADERFUNCTION, headers.write)
        try:
            connection.perform()
        finally:
            connection.close()

        content_ = content.getvalue()
        headers_ = dict(line.split(': ', 1) for line in headers.getvalue().decode().splitlines() if ': ' in line)

        return headers_, content_
