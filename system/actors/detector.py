from asyncio.tasks import sleep
import aiohttp
from icecream import ic
import helpers
from actors.camera_downloader import CameraDownloader
from actors.actor import Actor


class Detector(Actor):

    interval = 0

    def __init__(self):

        self.session = aiohttp.ClientSession(timeout=aiohttp.ClientTimeout(total=1))

    async def step(self, downloader: CameraDownloader):

        if not downloader.images:
            return

        image = downloader.images.pop()

        url = 'http://localhost:8004/detect'
        headers = {'mac': image.mac}
        data = {'file': image.data}

        helpers.measure()
        try:
            async with self.session.post(url, data=data, headers=headers) as response:
                detections = await response.json()
                ic(detections)
        except:
            helpers.print_stacktrace()
            # await self.sleep(0.1)
        finally:
            helpers.measure()
