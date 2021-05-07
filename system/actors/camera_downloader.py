import asyncio
from typing import List
import pycurl
import io
import logging
from pydantic.main import BaseModel
from actors.actor import Actor
from world.world import World
import helpers


class Image(BaseModel):

    time: float
    mac: str
    data: bytes


class CameraDownloader(Actor):

    interval: float = 0

    def __init__(self):

        self.images: List[Image] = []

    async def step(self, world: World):

        timeout = 1

        helpers.measure(reset=True)
        for camera in world.cameras.values():
            url = f'http://{camera.network.ip}:8080/image?maxtime={timeout}&killtime=2.5'
            loop = asyncio.get_event_loop()
            try:
                helpers.measure()
                start_time = world.time
                header, content = await loop.run_in_executor(None, self.get, url, timeout)
                end_time = world.time
                helpers.measure()
            except pycurl.error:
                logging.warning(f'image download error from {url}')
                continue
            # jpeg_header = simplejpeg.decode_jpeg_header(content)
            # ic(header)
            # ic(jpeg_header)
            time = (start_time + end_time) / 2.0  # TODO: improve accuracy via clock synchronization
            self.images.append(Image(time=time, mac=header['mac'], data=content))

        while self.images and self.images[0].time < world.time - 10.0:
            self.images.pop(0)

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
