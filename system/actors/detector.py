import aiohttp
from icecream import ic
import helpers
from actors.camera_downloader import CameraDownloader
from actors.actor import Actor


class Detector(Actor):

    interval = 0.2

    def __init__(self):

        self.session = aiohttp.ClientSession(timeout=aiohttp.ClientTimeout(total=0.5))

    async def step(self, downloader: CameraDownloader):

        if not downloader.images:
            return

        img = downloader.images.pop()
        headers = {'mac': img.mac}
        data = {'file': img.data}

        helpers.measure()
        try:
            async with self.session.post('http://localhost:8004/detect', data=data, headers=headers) as response:

                data = await response.json()
                ic(data)
        except:
            helpers.print_stacktrace()
        helpers.measure()
