import logging
from collections.abc import Awaitable, Callable

import httpx

from ... import rosys
from ..capture_device import CaptureDevice, CaptureState
from ..image import ImageArray
from .mjpeg_stream_worker import MjpegStreamWorker
from .stream_channel import EndReason, Frame, StreamEnded, StreamOpened
from .vendors import mac_to_url

ImageDataHandler = Callable[[ImageArray, float], Awaitable | None]
"""Receives a decoded frame together with its capture timestamp."""


class CameraAddressUnknown(Exception):
    """Raised when the camera settings are used before discovery has found an address."""


class CameraUnreachable(Exception):
    """Raised when the stream worker could not reach the camera."""


class MjpegDevice(CaptureDevice):

    def __init__(self, mac: str, ip: str | None = None, *,
                 index: int | None = None,
                 username: str | None = None,
                 password: str | None = None,
                 on_new_image_data: ImageDataHandler,
                 on_connect: Callable[[], Awaitable | None] | None = None,
                 reconnect_interval: float = 3.0) -> None:
        super().__init__(name=mac,
                         log=logging.getLogger('rosys.vision.mjpeg_camera.mjpeg_device.' + mac),
                         on_connect=on_connect,
                         reconnect_interval=reconnect_interval)
        self._mac = mac
        self._ip = ip
        self._index = index
        self._on_new_image_data = on_new_image_data
        self._username = username
        self._password = password
        self._worker: MjpegStreamWorker | None = None

        self._start_capture_task()

    @property
    def ip(self) -> str | None:
        """The address of the camera; assigning a new one makes the capture loop reopen the stream there."""
        return self._ip

    @ip.setter
    def ip(self, ip: str | None) -> None:
        if ip == self._ip:
            return
        self.log.info('address changed to %s', ip)
        self._ip = ip
        self.restart_capture()

    @property
    def url(self) -> str | None:
        """The stream URL, or ``None`` when the address is unknown or no URL scheme is known for the mac."""
        if self._ip is None:
            return None
        return mac_to_url(self._mac, self._ip, index=self._index)

    def _retry_reason(self) -> tuple[int, str] | None:
        if self._ip is None:
            return logging.DEBUG, 'no address known'
        if self.url is None:
            return logging.DEBUG, f'no stream URL for mac "{self._mac}"'
        if self.is_refused:
            return logging.INFO, 'camera refused the stream'
        return None

    def _describe_session_error(self, error: Exception) -> str:
        if isinstance(error, CameraAddressUnknown):
            return 'no address known yet'
        if isinstance(error, CameraUnreachable | httpx.HTTPError):
            return f'cannot reach the camera: {error}'
        return super()._describe_session_error(error)

    async def _prepare_stream(self) -> None:
        """Hook executed right before the MJPEG stream is opened (and on every restart).

        Vendors whose HTTP stream must be enabled before it serves data can override this.
        Implementations should log and return on failure rather than raise.
        """

    async def _run_session(self) -> None:
        """Have a worker process open the stream and consume its frames until it ends."""
        url = self.url
        if url is None:
            return
        self.log.debug('Starting capture task for %s', url)

        await self._prepare_stream()
        self._worker = MjpegStreamWorker(self._mac, url, self._username, self._password)
        try:
            while True:
                message = await self._worker.receive()
                if isinstance(message, StreamOpened):
                    await self._enter_streaming()
                elif isinstance(message, StreamEnded):
                    if self._keeps_running():  # a worker torn down by shutdown() ends its session on purpose
                        self._end_session(url, message)
                    return
                else:
                    if self.url != url:
                        self.log.info('stream settings changed; reopening the stream')
                        return
                    await self._deliver(message)
                    if not self._keeps_running():
                        return
        finally:
            await self._tear_down_session()

    def _end_session(self, url: str, message: StreamEnded) -> None:
        match message.reason:
            case EndReason.ENDED:
                self.log.debug('capture session ended')
            case EndReason.REFUSED:
                self.log.error('camera at %s refused the stream: %s', url, message.detail)
                self._set_state(CaptureState.REFUSED)
            case EndReason.UNREACHABLE:
                raise CameraUnreachable(message.detail)
            case EndReason.FAILED:
                raise RuntimeError(message.detail)

    async def _deliver(self, frame: Frame) -> None:
        timestamp = frame.capture_time if frame.capture_time is not None else rosys.time()
        try:
            callback_result = self._on_new_image_data(frame.array, timestamp)
            if isinstance(callback_result, Awaitable):
                await callback_result
        except Exception as e:  # pylint: disable=broad-exception-caught
            self.log.error('Error processing image: %s', e)

    async def _tear_down_session(self) -> None:
        if self._worker is None:
            return
        await self._worker.shutdown()
        self._worker = None

    async def get_fps(self) -> int | None:
        return None

    async def set_fps(self, fps: int) -> None:
        pass

    async def get_resolution(self) -> tuple[int, int] | None:
        return None

    async def set_resolution(self, width: int, height: int) -> None:
        pass

    async def get_mirrored(self) -> bool | None:
        return None

    async def set_mirrored(self, mirrored: bool) -> None:
        pass
