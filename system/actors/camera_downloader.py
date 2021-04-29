import asyncio
import pycurl
import io
import simplejpeg
from actors.actor import Actor
from world.world import World


class CameraDownloader(Actor):

    interval: float = 1.0

    async def step(self, world: World):

        timeout = 1

        for camera in world.cameras.values():
            url = f'http://{camera.network.ip}:8080/image?maxtime={timeout}&killtime=2.5'
            loop = asyncio.get_event_loop()
            try:
                header, content = await loop.run_in_executor(None, self.get, url, timeout)
            except pycurl.error:
                continue
            jpeg_header = simplejpeg.decode_jpeg_header(content)
            print(header, flush=True)
            print(jpeg_header, flush=True)

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
