import asyncio
import datetime
import logging
import multiprocessing
import sys

import cv2
import gi
import numpy as np

import rosys

from .jovision_rtsp_interface import JovisionInterface
from .vendors import VendorType, mac_to_url, mac_to_vendor

gi.require_version('Gst', '1.0')  # noqa
from gi.repository import GLib, Gst  # noqa

Gst.init(None)


class RtspDevice:

    def __init__(self, mac: str, ip: str, jovision_profile: int, fps: int = 10) -> None:
        self.log = logging.getLogger('rosys.vision.rtsp_camera.rtsp_device')

        self.mac = mac
        self.ip = ip

        self.fps = fps
        self.jovision_profile = jovision_profile

        self.capture_task: asyncio.Task | None = None
        self.capture_process: multiprocessing.Process | None = None
        self._image_buffer: bytes | None = None
        self._authorized: bool = True

        vendor_type = mac_to_vendor(mac)

        self.settings_interface: JovisionInterface | None = None
        if vendor_type == VendorType.JOVISION:
            self.settings_interface = JovisionInterface(ip)
        else:
            self.log.warning('[%s] No settings interface for vendor type %s', self.mac, vendor_type)
            self.log.warning('[%s] Using default fps of 10', self.mac)

        url = mac_to_url(mac, ip, jovision_profile)
        if url is None:
            raise ValueError(f'could not determine RTSP URL for {mac}')
        self.log.info('[%s] Starting VideoStream for %s', self.mac, url)
        self.parent_conn, self.child_conn = multiprocessing.Pipe()
        self.gstreamer_proc = None
        self._start_gstreamer_process()

    @property
    def authorized(self) -> bool:
        return self._authorized

    @property
    def url(self) -> str:
        url = mac_to_url(self.mac, self.ip, self.jovision_profile)
        if url is None:
            raise ValueError(f'could not determine RTSP URL for {self.mac}')
        return url

    def capture(self) -> bytes | None:
        if self.parent_conn.poll():
            print(f'[{self.mac}] RECV: {datetime.datetime.now().strftime("%H:%M:%S.%f")}', flush=True)
            data = self.parent_conn.recv()
            # create jpeg image bytes and return

            # Convert the received data to a numpy array
            nparr = np.frombuffer(data, np.uint8)

            # Decode the numpy array as an image
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

            # Encode the image as JPEG
            _, jpeg_data = cv2.imencode('.jpg', img)

            # Convert the JPEG data to bytes
            jpeg_bytes = jpeg_data.tobytes()

            return jpeg_bytes

        return None

    async def shutdown(self) -> None:
        if self.gstreamer_proc is not None:
            self.log.debug('[%s] Terminating gstreamer process', self.mac)
            self.gstreamer_proc.terminate()
            self.gstreamer_proc.join()
            self.gstreamer_proc = None

    def _start_gstreamer_process(self) -> None:
        print(f'[{self.mac}] Starting gstreamer process', flush=True)
        self.gstreamer_proc = multiprocessing.Process(target=self._run_gstreamer)
        self.gstreamer_proc.start()
        print(f'[{self.mac}] Gstreamer process started', flush=True)

    def _run_gstreamer(self):
        print(f"Stream {self.mac} - GStreamer process started")
        pipeline_str = str(
            # f'rtspsrc location="{self.url}" latency=0 protocols=tcp ! rtph264depay ! h264parse ! '
            # f'avdec_h264 ! videorate ! video/x-raw,framerate={self.fps}/1 ! videoconvert ! video/x-raw,format=BGR ! '
            # f'appsink name=appsink emit-signals=true drop=false max-buffers=1 sync=false'
            f'videotestsrc ! videoconvert ! video/x-raw,format=BGR ! appsink name=appsink emit-signals=true drop=false max-buffers=1 sync=false'
        )

        print(f"Stream {self.mac} - Pipeline: {pipeline_str}")
        pipeline = Gst.parse_launch(pipeline_str)

        appsink = pipeline.get_by_name("appsink")
        appsink.connect("new-sample", self._on_new_sample)

        print(f"Stream {self.mac} - Pipeline state set to PLAYING")
        pipeline.set_state(Gst.State.PLAYING)

        print(f"Stream {self.mac} - Starting Main Loop")
        loop = GLib.MainLoop()
        try:
            loop.run()
        except KeyboardInterrupt:
            pass
        finally:
            pipeline.set_state(Gst.State.NULL)

    def _on_new_sample(self, sink):
        print(f"Stream {self.mac} - on_new_sample called at {datetime.datetime.now().strftime('%H:%M:%S.%f')}")
        sys.stdout.flush()
        sample = sink.emit("pull-sample")
        if sample:
            buffer = sample.get_buffer()
            caps = sample.get_caps()

            print(f"Stream {self.mac} - Buffer pts: {buffer.pts}, dts: {buffer.dts}, duration: {buffer.duration}")

            if caps:
                structure = caps.get_structure(0)
                print(f"Stream {self.mac} - Caps: {structure.to_string()}")

            buffer_data = buffer.extract_dup(0, buffer.get_size())
            self.child_conn.send(buffer_data)

            print(
                f"Stream {self.mac} - SEND: {datetime.datetime.now().strftime('%H:%M:%S.%f')}: {len(buffer_data)} bytes", flush=True)
            return Gst.FlowReturn.OK
        else:
            print(f"Stream {self.mac} - No sample received")
            return Gst.FlowReturn.ERROR

    async def restart_gstreamer(self) -> None:
        print(f'[{self.mac}] Restarting gstreamer', flush=True)
        await self.shutdown()
        self._start_gstreamer_process()
        print(f'[{self.mac}] Gstreamer restarted', flush=True)

    async def set_fps(self, fps: int) -> None:
        self.fps = fps
        if self.settings_interface is not None:
            await self.settings_interface.set_fps(stream_id=self.jovision_profile, fps=self.fps)

    async def get_fps(self) -> int | None:
        if self.settings_interface is not None:
            return await self.settings_interface.get_fps(stream_id=self.jovision_profile)
        return self.fps

    def set_jovision_profile(self, profile: int) -> None:
        self.jovision_profile = profile

    def get_jovision_profile(self) -> int:
        return self.jovision_profile

    async def set_bitrate(self, bitrate: int) -> None:
        if self.settings_interface is not None:
            await self.settings_interface.set_bitrate(stream_id=self.jovision_profile, bitrate=bitrate)

    async def get_bitrate(self) -> int | None:
        if self.settings_interface is not None:
            return await self.settings_interface.get_bitrate(stream_id=self.jovision_profile)
        return None
