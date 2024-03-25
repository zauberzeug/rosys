import asyncio
import logging
import multiprocessing
from typing import Optional

import gi

from .jovision_rtsp_interface import JovisionInterface
from .vendors import VendorType, mac_to_url, mac_to_vendor

gi.require_version('Gst', '1.0')
from gi.repository import GLib, Gst  # noqa: E402

Gst.init(None)


def run_gstreamer_process(pipe_conn, url, fps):
    # pipeline_str = f'rtspsrc location="{url}" latency=0 protocols=tcp ! rtph264depay ! h264parse ! omxh264dec ! video/x-raw(memory:NVMM),format=NV12 ! nvvidconv ! video/x-raw,framerate={fps}/1,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! jpegenc ! appsink name=appsink emit-signals=true drop=true max-buffers=1'
    # cpu pipeline
    # pipeline_str = f'rtspsrc location="{url}" latency=0 protocols=tcp ! rtph264depay ! h264parse ! avdec_h264 ! jpegenc ! appsink name=appsink emit-signals=true drop=true max-buffers=1'
    # test pipeline with simple videosource
    pipeline_str = 'videotestsrc ! videoconvert ! video/x-raw,format=BGR ! jpegenc ! appsink name=appsink emit-signals=true drop=true max-buffers=1'
    pipeline = Gst.parse_launch(pipeline_str)
    appsink = pipeline.get_by_name("appsink")

    def on_new_sample(sink, _):
        sample = sink.emit("pull-sample")
        if sample:
            buffer = sample.get_buffer()
            buffer_data = buffer.extract_dup(0, buffer.get_size())
            pipe_conn.send(buffer_data)
        return Gst.FlowReturn.OK

    appsink.connect("new-sample", on_new_sample, None)
    pipeline.set_state(Gst.State.PLAYING)

    loop = GLib.MainLoop()
    loop.run()


class RtspDevice:
    def __init__(self, mac: str, ip: str, jovision_profile: int) -> None:
        self.mac = mac
        self.ip = ip
        self.jovision_profile = jovision_profile
        self.capture_task: Optional[asyncio.Task] = None
        self._image_buffer: Optional[bytes] = None
        self._authorized: bool = True
        self.url = None
        self.fps = 10  # Default FPS
        self.settings_interface = None
        self.log = logging.getLogger(f'rosys.rtsp_device.{mac}')
        self.log.setLevel(logging.DEBUG)

        # Example vendor handling
        vendor_type = mac_to_vendor(mac)
        if vendor_type == VendorType.JOVISION:
            self.settings_interface = JovisionInterface(ip)
            self.fps = self.settings_interface.get_fps(stream_id=jovision_profile)
        else:
            self.log.warning(f'No settings interface for vendor type {vendor_type}')
            self.log.warning('Using default fps of 10')

        self.url = mac_to_url(mac, ip, jovision_profile)
        if self.url is None:
            raise ValueError(f'Could not determine RTSP URL for {mac}')
        self.log.info(f'Starting VideoStream for {self.url}')

        self.parent_conn, self.child_conn = multiprocessing.Pipe()

        self.start_gstreamer_task()

    def set_jovision_profile(self, profile: int) -> None:
        self.jovision_profile = profile
        self.url = mac_to_url(self.mac, self.ip, profile)
        if self.url is None:
            raise ValueError(f'Could not determine RTSP URL for {self.mac}')
        # note: changes are only applied after a restart of the gstreamer process

    @property
    def authorized(self) -> bool:
        return self._authorized

    def capture(self) -> Optional[bytes]:
        image = self._image_buffer
        self._image_buffer = None
        return image

    def shutdown(self):
        if self.capture_process.is_alive():
            self.capture_process.terminate()
        self.capture_process.join()
        if self.capture_task and not self.capture_task.done():
            self.capture_task.cancel()

    def restart_gstreamer(self):
        self.shutdown()
        self.start_gstreamer_task()

    async def run_gstreamer(self):
        while True:
            if self.parent_conn.poll():
                image_data = self.parent_conn.recv()
                self._image_buffer = image_data
                self.log.debug('Got image data from gstreamer process (%d bytes)', len(image_data))
            await asyncio.sleep(0.1)  # Adjust as needed

    def start_gstreamer_task(self):
        self.log.info('Starting gstreamer process...')
        self.capture_process = multiprocessing.Process(
            target=run_gstreamer_process, args=(self.child_conn, self.url, self.fps))
        self.capture_process.start()

        if self.capture_task is None or self.capture_task.done():
            self.capture_task = asyncio.create_task(self.run_gstreamer())
