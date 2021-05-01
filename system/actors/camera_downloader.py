import asyncio
import pycurl
import io
import simplejpeg
import traceback
import requests
from functools import partial
from icecream import ic
from actors.actor import Actor
from world.world import World
import helpers


class CameraDownloader(Actor):

    interval: float = .2

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

            url = 'http://localhost:8004/detect'
            headers = {'mac': header['mac']}
            try:
                data = [('file', content)]
                helpers.measure()
                result = await loop.run_in_executor(None, partial(requests.post, url, files=data, headers=headers))
                helpers.measure()
                ic(result.content)
            except pycurl.error:
                # helpers.print_stacktrace()
                continue

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

    def post(self, url, timeout=1):

        content = io.BytesIO()

        connection = pycurl.Curl()
        connection.setopt(pycurl.TIMEOUT, timeout)
        connection.setopt(pycurl.URL, url)
        connection.setopt(pycurl.HTTPPOST, 1)
        connection.setopt(pycurl.WRITEFUNCTION, content.write)
        try:
            connection.perform()
        finally:
            connection.close()

        return content.getvalue()
