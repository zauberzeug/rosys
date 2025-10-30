# GStreamer initialization wrapper
from .vendors import VendorType, mac_to_url, mac_to_vendor
from .jovision_rtsp_interface import JovisionInterface
from ... import rosys
from nicegui import background_tasks, core
import sys

import gi as _gi  # underscore prefix prevents import sorting

# If gi.repository has not been imported yet, configure versions
if 'gi.repository' not in sys.modules:
    _gi.require_version('Gst', '1.0')
    _gi.require_version('GstApp', '1.0')

import asyncio
import logging
from collections.abc import Awaitable, Callable
from threading import Lock
from typing import Literal

import numpy as np
from gi.repository import Gst
print("gi import")


class AsyncRTPFrameReader:
    def __init__(self, url: str, avdec: str):
        print("before gst init")
        Gst.init(None)
        print(url)

        # Create pipeline
        pipeline_str = (
            f'rtspsrc location="{url}" latency=0 protocols=tcp ! '
            f'rtp{avdec}depay ! avdec_{avdec} ! videoconvert ! '
            'video/x-raw,format=RGB ! appsink name=sink sync=false'
        )
        print(pipeline_str)
        self.pipeline = Gst.parse_launch(pipeline_str)

        # Setup appsink
        sink_element = self.pipeline.get_by_name('sink')
        self.sink = sink_element.props
        self.sink.emit_signals = True
        self.sink.drop = True
        self.sink.max_buffers = 1

        # Setup synchronization primitives
        self._frame_ready = asyncio.Event()
        self._frame_lock = Lock()
        self._current_frame = None
        self._loop = core.loop

        # Setup callback
        sink_element.connect('new-sample', self._on_new_sample)

        # Start pipeline
        self.pipeline.set_state(Gst.State.PLAYING)

        print("pipeline started")

    def _on_new_sample(self, sink):
        try:
            sample = sink.emit('pull-sample')
            if not sample:
                return Gst.FlowReturn.ERROR

            buffer = sample.get_buffer()
            caps = sample.get_caps()

            structure = caps.get_structure(0)
            width = structure.get_value('width')
            height = structure.get_value('height')

            success, map_info = buffer.map(Gst.MapFlags.READ)
            if not success:
                return Gst.FlowReturn.ERROR

            try:
                frame = np.ndarray(
                    shape=(height, width, 3),
                    dtype=np.uint8,
                    buffer=map_info.data
                ).copy()

                # Update frame with proper synchronization
                with self._frame_lock:
                    self._current_frame = frame
                    self._loop.call_soon_threadsafe(self._frame_ready.set)

            finally:
                buffer.unmap(map_info)

            return Gst.FlowReturn.OK
        except Exception as e:
            print(f"Error in _on_new_sample: {e}")
            return Gst.FlowReturn.ERROR

    async def get_frame(self):
        """Get the next frame asynchronously"""
        await self._frame_ready.wait()
        print("waiting for frame done")
        self._frame_ready.clear()

        with self._frame_lock:
            assert self._current_frame is not None
            # TODO: Do we actually need to copy?
            return self._current_frame.copy()

    def __del__(self):
        if hasattr(self, 'pipeline'):
            self.pipeline.set_state(Gst.State.NULL)


class RtspDevice:
    def __init__(self, mac: str, ip: str, *,
                 substream: int, fps: int, on_new_image_data: Callable[[np.ndarray, float], Awaitable | None],
                 avdec: Literal['h264', 'h265'] = 'h264') -> None:
        self._mac = mac
        self._ip = ip
        self.log = logging.getLogger('rosys.vision.rtsp_camera.rtsp_device.' + self._mac)

        self._fps = fps
        self._substream = substream
        self._on_new_image_data = on_new_image_data
        self._avdec = avdec

        self._frame_reader: AsyncRTPFrameReader | None = None
        self._capture_task: asyncio.Task | None = None
        self._authorized: bool = True

        vendor_type = mac_to_vendor(mac)

        self._settings_interface: JovisionInterface | None = None
        if vendor_type == VendorType.JOVISION:
            self._settings_interface = JovisionInterface(ip)
        else:
            self.log.warning('[%s] No settings interface for vendor type %s', self._mac, vendor_type)
            self.log.warning('[%s] Using default fps of 10', self._mac)

        self.log.info('[%s] Starting VideoStream for %s', self._mac, self.url)
        self._start_gstreamer_task()

    @property
    def is_connected(self) -> bool:
        return self._capture_task is not None

    @property
    def authorized(self) -> bool:
        return self._authorized

    @property
    def url(self) -> str:
        url = mac_to_url(self._mac, self._ip, self._substream)
        if url is None:
            raise ValueError(f'could not determine RTSP URL for {self._mac}')
        return url

    async def shutdown(self) -> None:
        if self._frame_reader is not None:
            del self._frame_reader
            self._frame_reader = None

        if self._capture_task is not None and not self._capture_task.done():
            self.log.debug('[%s] Cancelling gstreamer task', self._mac)
            self._capture_task.cancel()
            try:
                await asyncio.wait_for(self._capture_task, timeout=5)
            except TimeoutError:
                self.log.warning('[%s] Timeout while waiting for capture task to cancel', self._mac)
                return
            except asyncio.CancelledError:
                self.log.debug('[%s] Task was successfully cancelled', self._mac)
            else:
                self.log.debug('[%s] Task finished', self._mac)
            self._capture_task = None

    def _start_gstreamer_task(self) -> None:
        self.log.debug('[%s] Starting gstreamer task', self._mac)
        if self._capture_task is not None and not self._capture_task.done():
            self.log.warning('[%s] capture task already running', self._mac)
            return
        print("start gstreamer task")
        self._capture_task = background_tasks.create(self._run_gstreamer(), name=f'capture {self._mac}')

    async def restart_gstreamer(self) -> None:
        await self.shutdown()
        self._start_gstreamer_task()

    async def _run_gstreamer(self) -> None:
        self._frame_reader = AsyncRTPFrameReader(self.url, self._avdec)

        try:
            while True:
                frame = await self._frame_reader.get_frame()
                timestamp = rosys.time()

                result = self._on_new_image_data(frame, timestamp)
                if isinstance(result, Awaitable):
                    await result

        except asyncio.CancelledError as e:
            self.log.info('[%s] stream ended', self._mac)
            print(e)
            raise

    async def set_fps(self, fps: int) -> None:
        self._fps = fps

        if self._settings_interface is not None:
            await self._settings_interface.set_fps(stream_id=self._substream, fps=self._fps)

    async def get_fps(self) -> int | None:
        if self._settings_interface is not None:
            return await self._settings_interface.get_fps(stream_id=self._substream)
        return self._fps

    def set_substream(self, index: int) -> None:
        self._substream = index

    def get_substream(self) -> int:
        return self._substream

    async def set_bitrate(self, bitrate: int) -> None:
        if self._settings_interface is not None:
            await self._settings_interface.set_bitrate(stream_id=self._substream, bitrate=bitrate)

    async def get_bitrate(self) -> int | None:
        if self._settings_interface is not None:
            return await self._settings_interface.get_bitrate(stream_id=self._substream)
        return None

    def get_avdec(self) -> Literal['h264', 'h265'] | None:
        return self._avdec

    def set_avdec(self, avdec: Literal['h264', 'h265']) -> None:
        self._avdec = avdec
