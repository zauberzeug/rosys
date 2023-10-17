import asyncio
import io
import logging
import shlex
import subprocess
import sys
from dataclasses import dataclass, field
from io import BytesIO
from typing import Any, AsyncGenerator, Optional

import cv2
import imgsize
import netifaces
import PIL

from .. import persistence, rosys
from ..geometry import Rectangle
from .camera import Camera, TransformCameraMixin
from .image import Image, ImageSize
from .image_processing import process_jpeg_image
from .image_rotation import ImageRotation


class PeekableBytesIO(io.BytesIO):

    def peek(self, n=-1):
        position = self.tell()
        data = self.read(n)
        self.seek(position)
        return data


def process_image(data: bytes, rotation: ImageRotation, crop: Optional[Rectangle] = None) -> bytes:
    image = PIL.Image.open(io.BytesIO(data))
    if crop is not None:
        image = image.crop((int(crop.x), int(crop.y), int(crop.x + crop.width), int(crop.y + crop.height)))
    image = image.rotate(int(rotation), expand=True)  # NOTE: PIL handles rotation with 90 degree steps efficiently
    img_byte_arr = io.BytesIO()
    image.save(img_byte_arr, format='JPEG')
    return img_byte_arr.getvalue()


async def find_ip_from_mac(mac: str) -> Optional[str]:
    """Find the IP address of a device with a given MAC address."""
    for interface in netifaces.interfaces():
        if sys.platform.startswith('darwin'):
            arp_cmd = 'sudo arp-scan'
        else:
            arp_cmd = 'sudo /usr/sbin/arp-scan'
        cmd = f'{arp_cmd} -I {interface} --localnet'
        print(cmd)
        output = await rosys.run.sh(cmd, timeout=10)
        print(output)
        if output is None:
            return None
        for line in output.splitlines():
            infos = line.split()
            if len(infos) < 2:
                continue
            if infos[1] == mac:
                return infos[0]
    return None


def determine_rtsp_url(mac: str, ip: str, jovision_profile: int = 0) -> Optional[str]:
    vendor_mac = mac[:8].lower()
    if vendor_mac == 'e0:62:90':  # Jovision IP Cameras
        return f'rtsp://admin:admin@{ip}/profile{jovision_profile}'
    if vendor_mac in ['e4:24:6c', '3c:e3:6b']:  # Dahua IP Cameras
        return f'rtsp://admin:Adminadmin@{ip}/cam/realmonitor?channel=1&subtype=0'
    return None


class RtspCameraOpenCvDevice:
    mac_address: str
    ip_address: str
    capture: Optional[cv2.VideoCapture] = None

    def __init__(self, mac, ip, jovision_profile=1) -> None:
        print(f'connecting to {mac} at {ip}', flush=True)
        self.ip_address = ip
        url = determine_rtsp_url(mac, self.ip_address, jovision_profile)
        if url is None:
            raise Exception(f'could not determine RTSP URL for {mac}')
        print('|||||||||||||||||||||||||||||||||', flush=True)
        print(f'connecting to {url}', flush=True)
        logging.info(f'Starting VideoStream for {url}')
        print('|||||||||||||||||||||||||||||||||', flush=True)
        self.capture = cv2.VideoCapture(url)
        if not self.capture.isOpened():
            raise Exception(f'could not connect to {url}')

    def url(self) -> str:
        return self.capture.get(cv2.CAP_PROP_POS_MSEC)

    def __del__(self) -> None:
        if self.capture is not None:
            self.capture.release()


class RtspCameraGstreamerDevice:
    mac: str
    ip_address: str
    url: str
    _image_buffer: Optional[bytes] = None
    capture_task: Optional[asyncio.Task] = None
    capture_process: Optional[subprocess.Popen] = None
    rotation: Optional[ImageRotation] = None
    crop: Optional[Rectangle] = None
    _authorized: bool = True
    resolution: Optional[ImageSize] = None

    def __init__(self, mac, ip, jovision_profile) -> None:
        self.mac = mac
        print(f'connecting to {mac} at {ip}', flush=True)
        self.ip_address = ip
        url = determine_rtsp_url(mac, self.ip_address, jovision_profile)
        if url is None:
            raise Exception(f'could not determine RTSP URL for {mac}')
        self.url = url
        print(f'connecting to {url}', flush=True)
        logging.info(f'Starting VideoStream for {url}')
        self.capture_task = rosys.background_tasks.create(self.start_gsreamer_task(url), name=f'capture {self.mac}')

    @property
    def authorized(self) -> bool:
        return self._authorized

    def capture(self) -> Optional[bytes]:
        image = self._image_buffer
        self._image_buffer = None
        return image

    def shutdown() -> None:
        if self.capture_process is not None:
            self.capture_process.terminate()
            self.capture_process = None

    async def start_gsreamer_task(self, url: str) -> None:
        async def stream(url: str) -> AsyncGenerator[bytes, None]:
            goal_fps = 10
            if 'subtype=0' in url:
                # to try: replace avdec_h264 with nvh264dec ! nvvidconv (!videoconvert)
                url = url.replace('subtype=0', 'subtype=1')
                command = f'gst-launch-1.0 rtspsrc location="{url}" latency=0 protocols=tcp ! rtph264depay ! avdec_h264 ! videoconvert ! videorate ! "video/x-raw,framerate={goal_fps}/1"! jpegenc ! fdsink'
            else:
                command = f'gst-launch-1.0 rtspsrc location="{url}" latency=0 protocols=tcp ! rtph264depay ! avdec_h264 ! videoconvert ! videorate ! "video/x-raw,framerate={goal_fps}/1" ! jpegenc ! fdsink'
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


@dataclass(slots=True, kw_only=True)
class RtspCamera(TransformCameraMixin, Camera):
    device: Optional[RtspCameraGstreamerDevice] = field(default=None, metadata=persistence.exclude)
    goal_fps: int = 6
    jovision_profile: int = 1

    detect: bool = False
    authorized: bool = True

    @property
    def is_connected(self) -> bool:
        return self.device is not None

    @property
    def url(self) -> Optional[str]:
        if self.is_connected:
            return self.device.url

    async def connect(self) -> None:
        if self.is_connected:
            return
        ip = await find_ip_from_mac(self.id)
        if ip is None:
            raise Exception(f'could not find IP address for {self.id}')

        device = RtspCameraGstreamerDevice(mac=self.id, ip=ip, jovision_profile=self.jovision_profile)
        if device is None:
            logging.warning(f'could not create device for {self.id}')
            return
        self.device = device

    async def disconnect(self) -> None:
        if not self.is_connected:
            return
        assert self.device is not None
        self.device.shutdown()
        self.device = None

    async def capture_image(self) -> Optional[Image]:
        if not self.is_connected:
            return None
        assert self.device is not None

        image = self.device.capture()
        if not image:
            return None
        bytes_ = await rosys.run.cpu_bound(process_jpeg_image, image, self.rotation, self.crop)

        self._add_image(Image(time=rosys.time(), camera_id=self.id, size=self.device.resolution, data=bytes_))

    @staticmethod
    def known_vendor(mac: str) -> bool:
        return mac[:8] in ['e0:62:90', 'e4:24:6c', '3c:e3:6b']
