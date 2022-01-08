import numpy as np

import cv2
import re

from rosys.helpers import measure
from .actor import Actor
from ..world.camera import Camera, Frame


class ImageCapture(Actor):
    interval: float = 0.30

    def __init__(self):
        super().__init__()
        self.devices = {}  # mapping camera ids to opencv devices

    async def step(self):
        await super().step()
        await self.update_device_list()
        for id, camera in self.world.cameras.items():
            if not camera.capture:
                return
            bytes = await self.run_io_bound(self.capture_frame, id)
            camera.frames.append(Frame(self.world.time, bytes))

    def capture_frame(self, id):
        _, frame = self.devices[id].read()
        bytes = cv2.imencode('.jpg', frame)[1].tobytes()
        return bytes

    async def update_device_list(self):
        output = await self.run_io_bound(self.run, ['v4l2-ctl', '--list-devices'])
        discovered = []
        for line in output.splitlines():
            if 'Camera' in line:
                id = re.search('\((.*)\)', line).group(1)
                if id in self.world.cameras:
                    camera = self.world.cameras[id]
                else:
                    self.world.cameras[id] = camera = Camera()
            if '/dev/video' in line:
                num = int(line.strip().lstrip('/dev/video'))
                if id not in self.devices:
                    self.devices[id] = self.get_capture_device(num)

    def get_capture_device(self, index: int):
        try:
            capture = cv2.VideoCapture(index)
            if capture is None or not capture.isOpened():
                self.log.error(f'{index} is unavailable device')
            else:
                return capture
        except:
            self.log.exception(f'{index} device failed')

    async def tear_down(self):
        await super().tear_down()
        for capture in self.devices.values():
            capture.release()
