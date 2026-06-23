from __future__ import annotations

import asyncio
import functools
import logging
import threading
from collections.abc import Awaitable, Callable
from typing import Any, Literal

import numpy as np
from nicegui import background_tasks

from ... import rosys
from ...vision.image import ImageArray
from .capture_time import DEFAULT_ABS_CAPTURE_TIME_EXT_ID, read_abs_capture_time
from .jovision_rtsp_interface import JovisionInterface
from .vendors import VendorType, mac_to_url, mac_to_vendor


@functools.lru_cache(maxsize=1)
def _load_gstreamer() -> Any:
    """Import and initialize the GStreamer Python bindings (PyGObject with ``Gst`` and ``GstRtp``).

    Returns the ``(Gst, GstRtp)`` modules, or ``None`` if the bindings are missing. The result is
    cached, so a deployment without them does not re-run the import or log the error on every
    reconnect attempt; it is reported once.
    """
    try:
        import gi  # pylint: disable=import-outside-toplevel  # noqa: PLC0415
        gi.require_version('Gst', '1.0')
        gi.require_version('GstRtp', '1.0')
        from gi.repository import Gst, GstRtp  # pylint: disable=import-outside-toplevel  # noqa: PLC0415
        Gst.init(None)
        return (Gst, GstRtp)
    except (ImportError, ValueError) as e:
        logging.getLogger('rosys.vision.rtsp_camera.rtsp_device').error(
            'RTSP capture requires the GStreamer Python bindings (PyGObject with Gst and GstRtp): %s', e)
        return None


class RtspDevice:

    def __init__(self, mac: str, ip: str, *,
                 substream: int, fps: int, on_new_image_data: Callable[[ImageArray, float], Awaitable | None],
                 avdec: Literal['h264', 'h265'] = 'h264') -> None:
        self._mac = mac
        self._ip = ip
        self.log = logging.getLogger('rosys.vision.rtsp_camera.rtsp_device.' + self._mac)

        self._fps = fps
        self._substream = substream
        self._on_new_image_data = on_new_image_data
        self._avdec = avdec
        self._ext_id = DEFAULT_ABS_CAPTURE_TIME_EXT_ID

        self._capture_task: asyncio.Task | None = None
        self._pipeline: Any = None
        self._gst: Any = None
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
        if self._pipeline is not None and self._gst is not None:
            self.log.debug('[%s] Stopping gstreamer pipeline', self._mac)
            self._pipeline.set_state(self._gst.State.NULL)
            self._pipeline = None
        if self._capture_task is not None and not self._capture_task.done():
            self.log.debug('[%s] Cancelling capture task', self._mac)
            self._capture_task.cancel()
            try:
                await asyncio.wait_for(self._capture_task, timeout=5)
            except asyncio.CancelledError:
                self.log.debug('[%s] Task was successfully cancelled', self._mac)
            except TimeoutError:
                self.log.warning('[%s] Timeout while waiting for capture task to cancel', self._mac)
        self._capture_task = None

    def _start_gstreamer_task(self) -> None:
        self.log.debug('[%s] Starting gstreamer task', self._mac)
        if self._capture_task is not None and not self._capture_task.done():
            self.log.warning('[%s] capture task already running', self._mac)
            return
        self._capture_task = background_tasks.create(self._run_gstreamer(), name=f'capture {self._mac}')

    async def restart_gstreamer(self) -> None:
        await self.shutdown()
        self._start_gstreamer_task()

    async def _run_gstreamer(self) -> None:
        if self._pipeline is not None:
            self.log.warning('[%s] pipeline already running', self._mac)
            return

        gstreamer = _load_gstreamer()
        if gstreamer is None:
            self._capture_task = None
            return
        Gst, GstRtp = gstreamer
        self._gst = Gst

        url = self.url
        # latency=0 + sync=false: emit frames as they arrive without client-side buffering
        # (the project values low delay over smoothing); the leaky queue keeps only the newest.
        # An appsink (rather than gdppay ! fdsink) lets us read the RTP header in a pad probe.
        pipeline_description = (
            f'rtspsrc location="{url}" protocols=tcp latency=0 name=src '
            f'! rtp{self._avdec}depay name=depay '
            f'! avdec_{self._avdec} '
            f'! videoconvert ! video/x-raw,format=RGB '
            f'! queue max-size-buffers=1 leaky=downstream '
            f'! appsink name=sink sync=false max-buffers=1 drop=true'
        )
        self.log.debug('[%s] Starting gstreamer pipeline for %s', self._mac, url)

        # capture_by_pts: GStreamer buffer PTS (ns) -> absolute capture epoch (s). Filled by the
        # RTP pad probe (runs on a streaming thread), read by the pull loop (runs in an executor
        # thread) -- so guard it with a lock. Bounded so it cannot grow without limit.
        capture_by_pts: dict[int, float] = {}
        pts_lock = threading.Lock()

        def on_rtp(_pad, info):
            buffer = info.get_buffer()
            if buffer is None or buffer.pts == Gst.CLOCK_TIME_NONE:
                return Gst.PadProbeReturn.OK
            ok, rtp = GstRtp.RTPBuffer.map(buffer, Gst.MapFlags.READ)
            if ok:
                capture_time = read_abs_capture_time(rtp, self._ext_id)
                rtp.unmap()
                # The extension rides only the first packet of each access unit, but every packet
                # of the unit shares this buffer PTS, so the first packet's value covers the unit.
                if capture_time is not None:
                    with pts_lock:
                        capture_by_pts[buffer.pts] = capture_time
                        if len(capture_by_pts) > 512:     # keep the newest ~256 entries
                            for old in list(capture_by_pts)[:256]:
                                del capture_by_pts[old]
            return Gst.PadProbeReturn.OK

        def to_array(sample) -> ImageArray | None:
            structure = sample.get_caps().get_structure(0)
            ok_w, width = structure.get_int('width')
            ok_h, height = structure.get_int('height')
            if not (ok_w and ok_h):
                return None
            buffer = sample.get_buffer()
            ok, info = buffer.map(Gst.MapFlags.READ)
            if not ok:
                return None
            try:
                if info.size != width * height * 3:
                    return None
                return np.frombuffer(info.data, dtype=np.uint8).reshape(height, width, 3).copy()
            finally:
                buffer.unmap(info)

        pipeline = Gst.parse_launch(pipeline_description)
        self._pipeline = pipeline
        pipeline.get_by_name('depay').get_static_pad('sink').add_probe(Gst.PadProbeType.BUFFER, on_rtp)
        sink = pipeline.get_by_name('sink')
        bus = pipeline.get_bus()

        if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            self.log.error('[%s] failed to start gstreamer pipeline', self._mac)
            pipeline.set_state(Gst.State.NULL)
            self._pipeline = None
            self._capture_task = None
            return

        loop = asyncio.get_event_loop()
        try:
            while True:
                sample = await loop.run_in_executor(None, sink.try_pull_sample, Gst.SECOND)
                if sample is None:
                    message = bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS)
                    if message is None:
                        continue   # pull timed out without a frame -- keep waiting
                    if message.type == Gst.MessageType.ERROR:
                        error, _ = message.parse_error()
                        self.log.error('[%s] gstreamer error: %s', self._mac, error.message)
                        if 'Unauthorized' in error.message:
                            self._authorized = False
                    else:
                        self.log.debug('[%s] end of stream', self._mac)
                    break

                frame = to_array(sample)
                if frame is None:
                    continue
                with pts_lock:
                    capture_time = capture_by_pts.get(sample.get_buffer().pts)
                timestamp = capture_time if capture_time is not None else rosys.time()
                result = self._on_new_image_data(frame, timestamp)
                if isinstance(result, Awaitable):
                    await result
        finally:
            pipeline.set_state(Gst.State.NULL)
            self._pipeline = None
            self._capture_task = None

        self.log.info('[%s] stream ended', self._mac)

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
