import asyncio
import logging
import shlex
import subprocess
from asyncio.subprocess import Process
from io import BytesIO
from typing import AsyncGenerator, Optional

from nicegui import background_tasks

from .jovision_rtsp_interface import JovisionInterface
from .vendors import VendorType, mac_to_url, mac_to_vendor


class RtspDevice:

    def __init__(self, mac: str, ip: str, jovision_profile: int) -> None:
        self.mac = mac

        self.capture_task: Optional[asyncio.Task] = None
        self.capture_process: Optional[Process] = None
        self._image_buffer: Optional[bytes] = None
        self._authorized: bool = True

        vendor_type = mac_to_vendor(mac)

        self.settings_interface: Optional[JovisionInterface] = None
        if vendor_type == VendorType.JOVISION:
            self.settings_interface = JovisionInterface(ip)
            self.fps = self.settings_interface.get_fps(stream_id=jovision_profile)
        else:
            logging.warning(f'no settings interface for vendor type {vendor_type}')
            logging.warning('using default fps of 10')
            self.fps = 10

        url = mac_to_url(mac, ip, jovision_profile)
        if url is None:
            raise ValueError(f'could not determine RTSP URL for {mac}')
        self.url = url
        logging.info(f'Starting VideoStream for {self.url}')
        self.start_gstreamer_task()

    @property
    def authorized(self) -> bool:
        return self._authorized

    def capture(self) -> Optional[bytes]:
        image = self._image_buffer
        self._image_buffer = None
        return image

    def shutdown(self) -> None:
        if self.capture_process is not None:
            self.capture_process.terminate()
            self.capture_process = None

    def start_gstreamer_task(self) -> None:
        self.capture_task = background_tasks.create(self.run_gstreamer(self.url), name=f'capture {self.mac}')

    def restart_gstreamer(self) -> None:
        self.shutdown()
        self.start_gstreamer_task()

    async def run_gstreamer(self, url: str) -> None:
        async def stream(url: str) -> AsyncGenerator[bytes, None]:
            if 'subtype=0' in url:
                url = url.replace('subtype=0', 'subtype=1')

            # to try: replace avdec_h264 with nvh264dec ! nvvidconv (!videoconvert)
            command = f'gst-launch-1.0 rtspsrc location="{url}" latency=0 protocols=tcp ! rtph264depay ! avdec_h264 ! videoconvert ! videorate ! "video/x-raw,framerate={self.fps}/1" ! jpegenc ! fdsink'
            process = await asyncio.create_subprocess_exec(*shlex.split(command), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            assert process.stdout is not None
            assert process.stderr is not None
            self.capture_process = process

            buffer = BytesIO()
            pos = 0
            header = None

            while process.returncode is None:
                assert process.stdout is not None
                new = await process.stdout.read(4096)
                if not new:
                    break
                buffer.write(new)

                img_range = None
                while True:
                    if header is None:
                        h = buffer.getvalue().find(b'\xff\xd8', pos)
                        if h == -1:
                            pos = buffer.tell() - 1
                            break
                        pos = h + 2
                        header = h
                    else:
                        f = buffer.getvalue().find(b'\xff\xd9', pos)
                        if f == -1:
                            pos = buffer.tell() - 1
                            break
                        img_range = (header, f + 2)
                        pos = f + 2
                        header = None

                if img_range:
                    yield buffer.getvalue()[img_range[0]:img_range[1]]

                    rest = buffer.getvalue()[img_range[1]:]
                    buffer.seek(0)
                    buffer.truncate()
                    buffer.write(rest)

                    pos = 0
                    if header is not None:
                        header -= img_range[1]

            assert process.stderr is not None
            error = await process.stderr.read()
            logging.info(f'process {process.pid} exited with {process.returncode} and error {error.decode()}')
            if 'Unauthorized' in error.decode():
                self._authorized = False

        async for image in stream(url):
            self._image_buffer = image

        self.capture_task = None
