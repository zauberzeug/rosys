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


class RtspDevice:

    def __init__(self, mac: str, ip: str, jovision_profile: int, fps: int = 10) -> None:
        self.log = logging.getLogger('rosys.vision.rtsp_camera.rtsp_device')

        self.mac = mac
        self.ip = ip

        self.fps = fps
        self.jovision_profile = jovision_profile

        self.capture_task: asyncio.Task | None = None
        self.capture_process: multiprocessing.Process | None = None
        self.ipc_queue = multiprocessing.Queue(maxsize=1)
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
        # url = "test"
        if url is None:
            raise ValueError(f'could not determine RTSP URL for {mac}')
        self.log.info('[%s] Starting VideoStream for %s', self.mac, url)
        self.parent_conn, self.child_conn = multiprocessing.Pipe()
        self.gstreamer_proc: multiprocessing.Process | None = None
        self.main_loop = None
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
        if not self.ipc_queue.empty():
            print(f'[{self.mac}] RECV: {datetime.datetime.now().strftime("%H:%M:%S.%f")}', flush=True)
            data = self.ipc_queue.get()
            return data

        return None

    async def shutdown(self) -> None:
        if self.gstreamer_proc is not None:
            self.log.debug('[%s] Terminating gstreamer process', self.mac)
            print(f'[{self.mac}] Terminating gstreamer process', flush=True)
            self.gstreamer_proc.terminate()
            self.gstreamer_proc.join()
            self.gstreamer_proc = None

    def _start_gstreamer_process(self) -> None:
        self.gstreamer_proc = multiprocessing.Process(target=self._run_gstreamer)
        self.gstreamer_proc.start()

    def _run_gstreamer(self):
        Gst.init(None)
        pipeline_str = str(
            f'rtspsrc location="{self.url}" latency=200 protocols=tcp ! rtph264depay ! h264parse ! '
            f'decodebin ! jpegenc ! appsink name=appsink emit-signals=true'

            # f'rtspsrc location="{self.url}" ! appsink name=appsink emit-signals=true '

            # f'videotestsrc ! video/x-raw,format=BGR,width=640,height=480,framerate={self.fps}/1 !'
            # ' timeoverlay !'
            # ' jpegenc ! appsink name=appsink emit-signals=true drop=true max-buffers=1 sync=true'

            # 'v4l2src ! videoconvert ! video/x-raw,format=BGR ! jpegenc ! appsink name=appsink emit-signals=true'
        )

        # self.log.debug(f'[{self.mac}] Started Gstreamer pipeline: {pipeline_str}')
        print(f'[{self.mac}] Started Gstreamer pipeline: {pipeline_str}', flush=True)
        pipeline = Gst.parse_launch(pipeline_str)

        appsink = pipeline.get_by_name("appsink")
        appsink.connect("new-sample", self._on_new_sample)
        bus = pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)

        pipeline.set_state(Gst.State.PLAYING)

        self.main_loop = GLib.MainLoop()
        print(f"Stream {self.mac} - Main loop created", flush=True)
        try:
            self.main_loop.run()
            print(f"Stream {self.mac} - Main loop exited", flush=True)
        except KeyboardInterrupt:
            pass
        finally:
            pipeline.set_state(Gst.State.NULL)
            print(f"Stream {self.mac} - GStreamer pipeline set to NULL", flush=True)

    def _on_bus_message(self, bus, message: Gst.Message):
        msg_type = message.type
        print(f"Stream {self.mac} - Bus message type: {msg_type}, source: {message.src.get_name()}", flush=True)
        if msg_type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            # self.log.error(f'[{self.mac}] GStreamer Error: {err}, {debug}')
            print(f'[{self.mac}] GStreamer Error: {err}, {debug}', flush=True)
            # Encerrar o loop principal
            GLib.idle_add(lambda: self._quit_main_loop())
        elif msg_type == Gst.MessageType.EOS:
            # self.log.info(f'[{self.mac}] GStreamer End-Of-Stream reached')
            print(f'[{self.mac}] GStreamer End-Of-Stream reached', flush=True)
            # Encerrar o loop principal
            GLib.idle_add(lambda: self._quit_main_loop())

    def _quit_main_loop(self):
        try:
            assert self.main_loop is not None
            self.main_loop.quit()
        except Exception as e:
            # self.log.error(f'[{self.mac}] Error quitting main loop: {e}')
            print(f'[{self.mac}] Error quitting main loop: {e}', flush=True)

    def _on_new_sample(self, sink):
        print(f"Stream {self.mac} - on_new_sample called at {datetime.datetime.now().strftime('%H:%M:%S.%f')}")
        sys.stdout.flush()
        sample = sink.emit("pull-sample")
        if sample:
            buffer = sample.get_buffer()
            caps = sample.get_caps()

            print(f"Stream {self.mac} - Buffer pts: {buffer.pts}, dts: {buffer.dts}, size: {buffer.get_size()}")

            buffer_data = buffer.extract_dup(0, buffer.get_size())
            try:
                self.ipc_queue.put_nowait(buffer_data)
            except multiprocessing.queues.Full:
                # print(f"Stream {self.mac} - Queue is full, dropping frame")
                pass

            # print(f"Stream {self.mac} - SEND: {datetime.datetime.now().strftime('%H:%M:%S.%f')}: {len(buffer_data)} bytes", flush=True)
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
