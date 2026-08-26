import asyncio
import gc
import logging
import weakref
from collections.abc import Awaitable
from contextlib import asynccontextmanager, suppress
from unittest.mock import AsyncMock, patch

import cv2
import numpy as np
import pytest

import rosys
import rosys.rosys as rosys_core
from rosys.testing import forward
from rosys.vision import (
    ImageSize,
    MjpegCamera,
    MjpegCameraProvider,
    RtspCamera,
    RtspCameraProvider,
    SimulatedCamera,
    SimulatedCameraProvider,
    UsbCamera,
    UsbCameraProvider,
)
from rosys.vision.camera.camera import MIN_RECONNECT_INTERVAL
from rosys.vision.mjpeg_camera.axis_mjpeg_device import AxisMjpegDevice
from rosys.vision.mjpeg_camera.mjpeg_device import CaptureState, MjpegDevice, StreamOpenResult
from rosys.vision.rtsp_camera.rtsp_device import RtspDevice
from rosys.vision.simulated_camera.simulated_device import SimulatedDevice
from rosys.vision.usb_camera.usb_device import UsbDevice

# A MAC that maps to a "GOODCAM" URL in both the RTSP and MJPEG vendor tables.
# GOODCAM needs no settings interface, so the device is constructed without any network access.
GOODCAM_MAC = '2c:6f:51:00:00:01'

# A MAC that maps to AXIS, whose stream settings are part of the URL rather than a settings interface.
AXIS_MAC = '00:40:8c:00:00:01'


async def _no_stream(self) -> None:
    """Stand-in for a capture session that never delivers a frame and never ends on its own."""
    await rosys.sleep(60.0)


def stalled_mjpeg_stream():
    """Keep an `MjpegDevice` in a single capture session without touching the network."""
    return patch.object(MjpegDevice, '_connect_and_stream_images', _no_stream)


def stalled_rtsp_stream():
    """Keep an `RtspDevice` in a single capture session without spawning gstreamer."""
    return patch.object(RtspDevice, '_run_gstreamer', _no_stream)


class FakeGstreamerProcess:
    """Stand-in for the gstreamer process, so a device reports `is_connected` without spawning one."""

    def __init__(self) -> None:
        self.returncode: int | None = None

    def terminate(self) -> None:
        self.returncode = -15

    async def wait(self) -> int | None:
        return self.returncode


async def _connected_rtsp_stream(self) -> None:
    """Stand-in for a gstreamer session that comes up and reapplies parameters, but spawns nothing."""
    self._capture_process = FakeGstreamerProcess()  # pylint: disable=protected-access
    try:
        await self._invoke_on_connect()  # pylint: disable=protected-access
        await rosys.sleep(60.0)
    finally:
        self._capture_process = None  # pylint: disable=protected-access


def connected_rtsp_stream():
    """Keep an `RtspDevice` in a single *connected* capture session without spawning gstreamer."""
    return patch.object(RtspDevice, '_run_gstreamer', _connected_rtsp_stream)


async def forward_until(condition, *, step: float = 0.3, real_step: float = 0.05,
                        attempts: int = 20, message: str = 'condition was not met') -> None:
    """Advance simulated time in steps, yielding real time between them, until `condition` holds.

    `rosys.testing.forward(until=...)` only yields via `asyncio.sleep(0)`, so it can exhaust its
    timeout before real loopback sockets or `io_bound` threads have made any progress.
    """
    for _ in range(attempts):
        if condition():
            return
        await forward(step)
        await asyncio.sleep(real_step)
    assert condition(), message


JPEG_FRAME = b'\xff\xd8' + bytes(32) + b'\xff\xd9'  # minimal SOI..EOI JPEG marker pair


@pytest.fixture
def vision_log(rosys_integration: None, caplog: pytest.LogCaptureFixture):
    logger = logging.getLogger('rosys.vision')  # NOTE: the test log configuration disables propagation to the root
    logger.addHandler(caplog.handler)
    yield caplog
    logger.removeHandler(caplog.handler)


async def shutdown_device(device) -> None:
    """Shut a device down, whichever of the two shapes its `shutdown` has."""
    result = device.shutdown()
    if isinstance(result, Awaitable):
        await result


async def wait_in_real_time(condition, *, step: float = 0.02, attempts: int = 200,
                            message: str = 'condition was not met') -> None:
    """Wait for `condition` without advancing simulated time.

    `forward()` does not advance while a `rosys.run.cpu_bound` call is in flight, so a test that
    holds one open has to wait in real time.
    """
    for _ in range(attempts):
        if condition():
            return
        await asyncio.sleep(step)
    assert condition(), message


def live_capture_tasks(name: str) -> list[asyncio.Task]:
    """The capture loops still running under this task name; a device names its task, so a leftover shows up."""
    return [task for task in asyncio.all_tasks() if task.get_name() == name and not task.done()]


async def cancel_leftover_loops(name: str) -> None:
    """Keep a failing test from leaving a capture loop behind that would hold up the next one."""
    for task in live_capture_tasks(name):
        task.cancel()
    await asyncio.sleep(0)


class SlowFirstDecode:
    """Stand-in for `nicegui.run.cpu_bound` that stalls its first call.

    `rosys.run.cpu_bound` turns a cancellation during the call into a `None` result instead of
    raising, so this is where a shutdown or an address change lands while a frame is being decoded.
    """

    def __init__(self, seconds: float = 0.3) -> None:
        self.seconds = seconds
        self.started = asyncio.Event()
        self.finished = asyncio.Event()

    async def __call__(self, callback, *args, **kwargs):
        if not self.started.is_set():
            self.started.set()
            try:
                await asyncio.sleep(self.seconds)
            finally:  # the cancellation arrives here, so the stall ends either way
                self.finished.set()
        return callback(*args, **kwargs)


async def decode_frame(data: bytes, timestamp: float) -> None:  # pylint: disable=unused-argument
    """Image callback shaped like `MjpegCamera._handle_new_image_data`, which decodes via `cpu_bound`."""
    await rosys.run.cpu_bound(len, data)


class FlakyMjpegServer:
    """Local HTTP server that serves `frames_per_connection` fake JPEG frames and then drops the connection.

    One frame per connection mimics a camera whose stream keeps ending; `None` keeps the stream open.
    """

    def __init__(self, frames_per_connection: int | None = 1) -> None:
        self.connections = 0
        self.frames_per_connection = frames_per_connection
        self._server: asyncio.Server | None = None

    @property
    def port(self) -> int:
        assert self._server is not None
        return self._server.sockets[0].getsockname()[1]

    async def start(self) -> None:
        self._server = await asyncio.start_server(self._handle, '127.0.0.1', 0)

    async def _handle(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        self.connections += 1
        try:
            try:
                await asyncio.wait_for(reader.readuntil(b'\r\n\r\n'), timeout=2)
            except Exception:  # reading the request is best-effort
                pass
            writer.write(b'HTTP/1.1 200 OK\r\n'
                         b'Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n')
            sent = 0
            while self.frames_per_connection is None or sent < self.frames_per_connection:
                writer.write(JPEG_FRAME)
                await writer.drain()
                sent += 1
                if self.frames_per_connection is None:
                    await asyncio.sleep(0.02)
        except (ConnectionResetError, BrokenPipeError):
            pass  # the device closed the stream
        finally:
            writer.close()

    async def stop(self) -> None:
        assert self._server is not None
        self._server.close()
        with suppress(TimeoutError):  # a client still reading must not hold up the test
            await asyncio.wait_for(self._server.wait_closed(), timeout=1)


class Unauthorized401Server:
    """Local HTTP server that always responds 401 and closes the connection."""

    def __init__(self) -> None:
        self.connections = 0
        self._server: asyncio.Server | None = None

    @property
    def port(self) -> int:
        assert self._server is not None
        return self._server.sockets[0].getsockname()[1]

    async def start(self) -> None:
        self._server = await asyncio.start_server(self._handle, '127.0.0.1', 0)

    async def _handle(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        self.connections += 1
        try:
            try:
                await asyncio.wait_for(reader.readuntil(b'\r\n\r\n'), timeout=2)
            except Exception:  # reading the request is best-effort
                pass
            writer.write(b'HTTP/1.1 401 Unauthorized\r\n'
                         b'Content-Length: 0\r\n'
                         b'Connection: close\r\n\r\n')
            await writer.drain()
        finally:
            writer.close()

    async def stop(self) -> None:
        assert self._server is not None
        self._server.close()
        await self._server.wait_closed()


async def test_mjpeg_device_reconnects_after_stream_drops(rosys_integration):
    server = FlakyMjpegServer()
    await server.start()
    frames: list[bytes] = []
    device = MjpegDevice(GOODCAM_MAC, f'127.0.0.1:{server.port}',
                         on_new_image_data=lambda data, timestamp: frames.append(data))
    device.reconnect_interval = 0.2
    try:
        await forward_until(lambda: server.connections >= 3, attempts=60,
                            message='device did not reconnect')
        await forward_until(lambda: len(frames) >= 3, attempts=60,
                            message='no frames received across reconnects')

        device.shutdown()
        await asyncio.sleep(0.1)
        assert not device.is_connected
        connections_at_shutdown = server.connections
        for _ in range(5):
            await forward(0.3)
            await asyncio.sleep(0.05)
        assert server.connections == connections_at_shutdown, 'device kept reconnecting after shutdown'
    finally:
        device.shutdown()
        await server.stop()


async def test_rtsp_device_reconnects_until_shutdown(rosys_integration):
    sessions = 0

    async def fake_gstreamer(self) -> None:
        nonlocal sessions
        sessions += 1
        await rosys.sleep(0.05)  # simulate a short-lived stream that ends on its own

    with patch.object(RtspDevice, '_run_gstreamer', fake_gstreamer):
        device = RtspDevice(GOODCAM_MAC, '192.168.0.5', substream=0, fps=5,
                            on_new_image_data=lambda array, timestamp: None,
                            reconnect_interval=0.2)
        await forward(2.0)
        assert sessions >= 3, f'device did not reconnect (only {sessions} sessions)'
        assert device.is_active  # loop alive; is_connected is False here because the stubbed gstreamer opens no process

        await device.shutdown()
        assert not device.is_active
        sessions_at_shutdown = sessions
        await forward(2.0)
        assert sessions == sessions_at_shutdown, 'device kept reconnecting after shutdown'


def test_rtsp_camera_persists_reconnect_interval(rosys_integration):
    camera = RtspCamera(mac='aa:bb:cc:dd:ee:ff', reconnect_interval=12.5, connect_after_init=False)
    data = camera.to_dict()
    assert data['reconnect_interval'] == 12.5
    restored = RtspCamera.from_dict(data)
    assert restored.reconnect_interval == 12.5


def test_mjpeg_camera_persists_reconnect_interval(rosys_integration):
    camera = MjpegCamera(id='test_cam', ip='192.168.1.1', reconnect_interval=12.5, connect_after_init=False)
    data = camera.to_dict()
    assert data['reconnect_interval'] == 12.5
    restored = MjpegCamera.from_dict(data)
    assert restored.reconnect_interval == 12.5


async def test_rtsp_camera_passes_reconnect_interval_to_device(rosys_integration):
    camera = RtspCamera(mac=GOODCAM_MAC, ip='192.168.0.5', reconnect_interval=7.0, connect_after_init=False)
    with stalled_rtsp_stream():
        await camera.connect()
        assert camera.device is not None
        assert camera.device.reconnect_interval == 7.0
        await camera.disconnect()


async def test_mjpeg_camera_passes_reconnect_interval_to_device(rosys_integration):
    camera = MjpegCamera(id=GOODCAM_MAC, ip='127.0.0.1:1', reconnect_interval=7.0, connect_after_init=False)
    with stalled_mjpeg_stream():
        await camera.connect()
        assert camera.device is not None
        assert camera.device.reconnect_interval == 7.0
        await camera.disconnect()


class FakeCapture:
    """Minimal stand-in for cv2.VideoCapture that can simulate a disconnected device."""

    def __init__(self) -> None:
        self._opened = True
        self._frame = np.zeros((48, 64, 3), dtype=np.uint8)
        self.props: dict = {}

    def isOpened(self) -> bool:  # cv2 API name
        return self._opened

    def read(self):
        if not self._opened:
            return False, None
        return True, self._frame

    def release(self) -> None:
        self._opened = False

    def get(self, prop) -> float:
        return self.props.get(prop, 0.0)

    def set(self, prop, value) -> bool:
        self.props[prop] = value
        return True


async def test_usb_device_reconnects_after_disconnect(rosys_integration):
    captures: list[FakeCapture] = []

    def make_capture(_device_node: str) -> FakeCapture:
        capture = FakeCapture()
        captures.append(capture)
        return capture

    frames: list = []
    with patch.object(UsbDevice, 'create_capture', make_capture), \
            patch('rosys.vision.usb_camera.usb_device.find_device_node', return_value='/dev/video0'):
        device = UsbDevice('fakecam', on_new_image_data=lambda data, timestamp: frames.append(data),
                           reconnect_interval=0.3)
        try:
            await forward_until(lambda: bool(frames), step=0.1, real_step=0.02, attempts=15,
                                message='device did not capture any frames')
            assert device.is_connected
            captures_before = len(captures)

            captures[-1].release()  # simulate a bad cable: the capture dies
            await forward_until(lambda: len(captures) > captures_before, real_step=0.02,
                                message='device did not reopen its capture')
            assert device.is_connected

            frames_after_reconnect = len(frames)
            await forward_until(lambda: len(frames) > frames_after_reconnect,
                                step=0.1, real_step=0.02, attempts=10,
                                message='frames did not resume after reconnect')

            await device.shutdown()
            assert not device.is_connected
            captures_at_shutdown = len(captures)
            for _ in range(5):
                await forward(0.3)
                await asyncio.sleep(0.02)
            assert len(captures) == captures_at_shutdown, 'device kept reopening after shutdown'
        finally:
            await device.shutdown()


async def test_usb_camera_reapplies_parameters_after_reconnect(rosys_integration):
    captures: list[FakeCapture] = []

    def make_capture(_device_node: str) -> FakeCapture:
        capture = FakeCapture()
        captures.append(capture)
        return capture

    with patch.object(UsbDevice, 'create_capture', make_capture), \
            patch('rosys.vision.usb_camera.usb_device.find_device_node', return_value='/dev/video0'):
        camera = UsbCamera(id='fakecam', width=640, height=480, fps=5,
                           connect_after_init=False, reconnect_interval=0.3)
        await camera.connect()
        try:
            await forward_until(lambda: bool(captures) and cv2.CAP_PROP_FRAME_WIDTH in captures[0].props,
                                step=0.1, real_step=0.02,
                                message='device did not open a capture and apply parameters')
            assert captures[0].props[cv2.CAP_PROP_FRAME_WIDTH] == 640, 'parameters were not applied on connect'

            captures[-1].release()  # simulate a bad cable: the capture dies
            await forward_until(lambda: len(captures) > 1 and cv2.CAP_PROP_FPS in captures[-1].props,
                                real_step=0.02,
                                message='device did not reopen its capture and reapply parameters')
            assert captures[-1].props[cv2.CAP_PROP_FRAME_WIDTH] == 640, 'width was not reapplied after reconnect'
            assert captures[-1].props[cv2.CAP_PROP_FRAME_HEIGHT] == 480, 'height was not reapplied after reconnect'
            assert captures[-1].props[cv2.CAP_PROP_FPS] == 5, 'fps was not reapplied after reconnect'
        finally:
            await camera.disconnect()


async def test_simulated_camera_reconnects_after_disconnect(rosys_integration):
    camera = SimulatedCamera(id='sim', width=64, height=48, fps=10, reconnect_interval=0.5)
    await camera.connect()
    assert camera.device is not None
    await forward(0.5)
    assert camera.images, 'no images while connected'

    camera.device.disconnect()  # simulate a bad cable
    assert not camera.device.is_connected
    count_at_disconnect = len(camera.images)
    await forward(0.3)  # shorter than reconnect_interval -> still disconnected
    assert not camera.device.is_connected
    assert len(camera.images) == count_at_disconnect, 'images kept arriving while disconnected'

    await forward(0.5)  # now past reconnect_interval since the disconnect
    assert camera.device.is_connected
    await forward(0.3)
    assert len(camera.images) > count_at_disconnect, 'images did not resume after reconnect'


async def test_simulated_camera_reapplies_parameters_after_reconnect(rosys_integration):
    camera = SimulatedCamera(id='sim', width=64, height=48, fps=10, color='#123456', reconnect_interval=0.5)
    await camera.connect()
    assert camera.device is not None
    await forward(0.5)

    camera.device.disconnect()  # simulate a bad cable
    await camera.set_parameters({'color': '#654321'})  # while disconnected, the new value only reaches the cache
    assert camera.device.color == '#123456'

    await forward(1.0)  # the device reconnects itself and the camera reapplies its parameters
    assert camera.device.is_connected
    assert camera.device.color == '#654321'


async def test_simulated_camera_passes_params_to_device(rosys_integration):
    camera = SimulatedCamera(id='sim', reconnect_interval=9.0, simulate_failing=True)
    await camera.connect()
    assert camera.device is not None
    assert camera.device.reconnect_interval == 9.0
    assert camera.device.simulate_failing is True
    await camera.disconnect()


def test_simulated_camera_persists_reconnect_interval(rosys_integration):
    camera = SimulatedCamera(id='sim', reconnect_interval=12.5, connect_after_init=False)
    data = camera.to_dict()
    assert data['reconnect_interval'] == 12.5
    assert SimulatedCamera.from_dict(data).reconnect_interval == 12.5


def test_usb_camera_persists_reconnect_interval(rosys_integration):
    camera = UsbCamera(id='cam', reconnect_interval=12.5, connect_after_init=False)
    data = camera.to_dict()
    assert data['reconnect_interval'] == 12.5
    assert UsbCamera.from_dict(data).reconnect_interval == 12.5


async def test_rtsp_is_active_distinct_from_is_connected(rosys_integration):
    sessions = 0

    async def fake_gstreamer(self) -> None:
        nonlocal sessions
        sessions += 1
        await rosys.sleep(0.05)

    with patch.object(RtspDevice, '_run_gstreamer', fake_gstreamer):
        device = RtspDevice(GOODCAM_MAC, '192.168.0.5', substream=0, fps=5,
                            on_new_image_data=lambda array, timestamp: None,
                            reconnect_interval=0.2)
        try:
            await forward(0.6)
            assert sessions >= 1, f'expected fake gstreamer to run at least once, got {sessions}'
            assert device.is_active is True, 'expected reconnect loop to stay alive while waiting between sessions'
            assert device.is_connected is False, 'expected no streaming process with fake gstreamer stub'
        finally:
            await device.shutdown()
        assert device.is_active is False, 'expected reconnect loop to stop after shutdown'
        assert device.is_connected is False, 'expected stream state to be disconnected after shutdown'


async def test_mjpeg_device_backs_off_after_401(rosys_integration):
    server = Unauthorized401Server()
    await server.start()
    device = MjpegDevice(GOODCAM_MAC, f'127.0.0.1:{server.port}',
                         on_new_image_data=lambda data, timestamp: None,
                         reconnect_interval=0.2)
    device.UNAUTHORIZED_RECONNECT_INTERVAL = 4.0  # type: ignore[misc]
    try:
        await forward_until(lambda: not device.authorized, step=0.2,
                            message='expected the device to mark itself unauthorized after a 401 response')
        assert device.is_active is True, 'expected the capture loop to stay alive while backing off'
        assert device.is_connected is False, 'expected unauthorized device to remain disconnected'

        connections_after_401 = server.connections
        for _ in range(5):  # well beyond reconnect_interval, still inside the unauthorized back-off
            await forward(0.2)
            await asyncio.sleep(0.05)
        assert server.connections == connections_after_401, (
            'expected the device to throttle its retries after a 401 unauthorized response'
        )

        await forward_until(lambda: server.connections > connections_after_401, step=0.5, attempts=40,
                            message='expected the device to retry once the unauthorized back-off elapsed')
    finally:
        device.shutdown()
        await server.stop()


async def test_mjpeg_device_retries_at_a_new_address_despite_back_off(rosys_integration):
    rejecting_server = Unauthorized401Server()
    new_server = FlakyMjpegServer()
    await rejecting_server.start()
    await new_server.start()
    device = MjpegDevice(GOODCAM_MAC, f'127.0.0.1:{rejecting_server.port}',
                         on_new_image_data=lambda data, timestamp: None,
                         reconnect_interval=0.2)
    device.UNAUTHORIZED_RECONNECT_INTERVAL = 600.0  # type: ignore[misc]
    try:
        await forward_until(lambda: not device.authorized, step=0.2,
                            message='expected the device to mark itself unauthorized after a 401 response')

        device.ip = f'127.0.0.1:{new_server.port}'  # a rejection was about the previous address
        await forward_until(lambda: new_server.connections >= 1, step=0.2,
                            message='expected an address change to end the unauthorized back-off')
    finally:
        device.shutdown()
        await rejecting_server.stop()
        await new_server.stop()


async def test_rtsp_device_backs_off_with_a_zero_reconnect_interval(rosys_integration):
    sessions = 0

    async def unauthorized_gstreamer(self) -> None:
        nonlocal sessions
        sessions += 1
        self._authorized = False  # pylint: disable=protected-access

    with patch.object(RtspDevice, '_run_gstreamer', unauthorized_gstreamer):
        device = RtspDevice(GOODCAM_MAC, '192.168.0.5', substream=0, fps=5,
                            on_new_image_data=lambda array, timestamp: None,
                            reconnect_interval=0.0)
        device.UNAUTHORIZED_RECONNECT_INTERVAL = 1.0  # type: ignore[misc]
        try:
            await forward(0.5)
            assert sessions == 1, f'expected the device to throttle its retries, got {sessions} sessions'

            await forward(1.0)
            assert sessions >= 2, 'expected the back-off to end instead of holding the loop forever'
        finally:
            await device.shutdown()


@pytest.mark.parametrize('interval', [0.0, -1.0])
async def test_devices_never_retry_without_a_delay(vision_log, interval: float):
    devices = [
        MjpegDevice(GOODCAM_MAC, on_new_image_data=lambda data, timestamp: None, reconnect_interval=interval),
        RtspDevice(GOODCAM_MAC, substream=0, fps=5,
                   on_new_image_data=lambda data, timestamp: None, reconnect_interval=interval),
        UsbDevice('no-such-camera', on_new_image_data=lambda data, timestamp: None, reconnect_interval=interval),
    ]
    try:
        for device in devices:
            assert device.reconnect_interval >= MIN_RECONNECT_INTERVAL, (
                f'{type(device).__name__} would reconnect without waiting'
            )
        complaints = [record for record in vision_log.records if 'too short' in record.getMessage()]
        assert len(complaints) == len(devices), 'expected every device to report the interval it cannot honor'
    finally:
        for device in devices:
            await shutdown_device(device)


async def test_a_reconnect_interval_that_can_be_honored_is_kept(vision_log):
    device = MjpegDevice(GOODCAM_MAC, on_new_image_data=lambda data, timestamp: None, reconnect_interval=2.5)
    try:
        assert device.reconnect_interval == 2.5
        assert not [record for record in vision_log.records if 'too short' in record.getMessage()]

        device.reconnect_interval = 0.0
        assert device.reconnect_interval == MIN_RECONNECT_INTERVAL, 'expected a later assignment to be held too'
        assert [record for record in vision_log.records if 'too short' in record.getMessage()]
    finally:
        device.shutdown()


async def test_rtsp_device_backs_off_after_rejected_login(rosys_integration):
    sessions = 0

    async def unauthorized_gstreamer(self) -> None:
        nonlocal sessions
        sessions += 1
        self._authorized = False  # pylint: disable=protected-access

    with patch.object(RtspDevice, '_run_gstreamer', unauthorized_gstreamer):
        device = RtspDevice(GOODCAM_MAC, '192.168.0.5', substream=0, fps=5,
                            on_new_image_data=lambda array, timestamp: None,
                            reconnect_interval=0.2)
        device.UNAUTHORIZED_RECONNECT_INTERVAL = 4.0  # type: ignore[misc]
        try:
            await forward(0.5)
            assert sessions == 1, f'expected the device to throttle its retries, got {sessions} sessions'
            assert device.authorized is False, 'expected the device to mark itself unauthorized'
            assert device.is_active is True, 'expected the capture loop to stay alive while backing off'

            await forward(4.0)
            assert sessions >= 2, 'expected the device to retry once the unauthorized back-off elapsed'

            device.ip = '192.168.0.6'  # a rejection was about the previous address
            sessions_at_address_change = sessions
            await forward(0.5)
            assert sessions > sessions_at_address_change, (
                'expected an address change to end the unauthorized back-off'
            )
        finally:
            await device.shutdown()


async def test_mjpeg_device_invokes_on_connect_per_session(rosys_integration):
    server = FlakyMjpegServer()
    await server.start()
    connect_calls = 0

    def count_connect() -> None:
        nonlocal connect_calls
        connect_calls += 1

    device = MjpegDevice(GOODCAM_MAC, f'127.0.0.1:{server.port}',
                         on_new_image_data=lambda data, timestamp: None,
                         on_connect=count_connect,
                         reconnect_interval=0.2)
    try:
        await forward_until(lambda: connect_calls >= 3,
                            message='on_connect was not invoked for each new stream')
    finally:
        device.shutdown()
        await server.stop()


async def test_rtsp_camera_restarts_stream_on_set_parameters_but_not_on_reapply(rosys_integration):
    camera = RtspCamera(mac=GOODCAM_MAC, ip='192.168.0.5', connect_after_init=False)
    with connected_rtsp_stream(), \
            patch.object(RtspDevice, 'restart_gstreamer') as restart:
        await camera.connect()
        # the session reapplies the parameters itself, which only runs the setters while connected
        await forward_until(lambda: camera.is_connected, message='expected the capture session to come up')

        restart.assert_not_called()  # a reapply from the device's own capture task must not restart the stream

        await camera.set_parameters({'fps': 7})
        restart.assert_called_once()
        await camera.disconnect()


async def test_axis_camera_applies_parameters_without_restarting_its_own_session(rosys_integration):
    sessions = 0

    async def stream(self) -> None:
        nonlocal sessions
        sessions += 1
        self._state = CaptureState.STREAMING  # pylint: disable=protected-access
        await self._invoke_on_connect()  # pylint: disable=protected-access
        await rosys.sleep(60.0)

    camera = MjpegCamera(id=AXIS_MAC, ip='192.168.0.5', fps=12, connect_after_init=False)
    with patch.object(MjpegDevice, '_connect_and_stream_images', stream):
        await camera.connect()
        try:
            await forward_until(lambda: sessions >= 1, message='expected a capture session to start')
            await forward(2.0)
            assert sessions == 1, 'expected the reapplied parameters not to restart the capture task'
            assert camera.device is not None
            assert 'fps=12' in (camera.device.url or ''), 'expected the requested fps to reach the stream URL'
        finally:
            await camera.disconnect()


async def test_mjpeg_device_reopens_the_stream_when_its_url_changes(rosys_integration):
    opened_urls: list[str] = []

    @asynccontextmanager
    async def open_stream(client, url, username, password):  # pylint: disable=unused-argument
        opened_urls.append(url)
        yield StreamOpenResult(object())  # the response only reaches the patched frame reader

    async def endless_frames(self, response):  # pylint: disable=unused-argument
        while True:
            await rosys.sleep(0.05)
            yield b'\xff\xd8' + bytes(32) + b'\xff\xd9', None

    with patch('rosys.vision.mjpeg_camera.mjpeg_device.open_stream', open_stream), \
            patch.object(MjpegDevice, '_frame_reader', endless_frames):
        device = AxisMjpegDevice(AXIS_MAC, '192.168.0.5', on_new_image_data=lambda data, timestamp: None)
        device.reconnect_interval = 0.2
        try:
            await forward_until(lambda: len(opened_urls) == 1, message='expected the stream to be opened')
            await device.set_fps(12)
            await forward_until(lambda: len(opened_urls) == 2,
                                message='expected the stream to reopen after the settings changed')
            assert 'fps=6' in opened_urls[0] and 'fps=12' in opened_urls[1]
        finally:
            device.shutdown()


async def test_mjpeg_device_stops_streaming_when_shut_down_during_a_callback(rosys_integration):
    server = FlakyMjpegServer(frames_per_connection=None)
    await server.start()
    frames: list[bytes] = []
    decode = SlowFirstDecode()

    async def handle_image(data: bytes, timestamp: float) -> None:
        await decode_frame(data, timestamp)
        frames.append(data)

    with patch('nicegui.run.cpu_bound', decode):
        device = MjpegDevice(GOODCAM_MAC, f'127.0.0.1:{server.port}', on_new_image_data=handle_image)
        try:
            await wait_in_real_time(decode.started.is_set, message='no frame reached the callback')
            device.shutdown()
            await wait_in_real_time(decode.finished.is_set, message='the stalled callback never returned')
            await asyncio.sleep(0.05)  # let the frame that was in flight finish
            frames_after_shutdown = len(frames)
            await asyncio.sleep(0.15)  # a surviving session would deliver several more frames
            assert len(frames) == frames_after_shutdown, 'the capture task kept streaming after shutdown'
            assert not live_capture_tasks(f'mjpeg capture {GOODCAM_MAC}'), 'the capture task survived shutdown'
        finally:
            await cancel_leftover_loops(f'mjpeg capture {GOODCAM_MAC}')
            await server.stop()


async def test_mjpeg_device_keeps_one_capture_loop_across_an_address_change(rosys_integration):
    server = FlakyMjpegServer(frames_per_connection=None)
    await server.start()
    decode = SlowFirstDecode()

    with patch('nicegui.run.cpu_bound', decode):
        device = MjpegDevice(GOODCAM_MAC, f'127.0.0.1:{server.port}', on_new_image_data=decode_frame)
        try:
            await wait_in_real_time(decode.started.is_set, message='no frame reached the callback')
            device.ip = '127.0.0.1:1'  # a provider rebinds the camera to another address
            await wait_in_real_time(decode.finished.is_set, message='the stalled callback never returned')
            await asyncio.sleep(0.1)
            assert len(live_capture_tasks(f'mjpeg capture {GOODCAM_MAC}')) == 1, \
                'the replaced capture loop is still running'
        finally:
            device.shutdown()
            await cancel_leftover_loops(f'mjpeg capture {GOODCAM_MAC}')
            await server.stop()


async def _stub_usb_session(self) -> bool:
    """Stand-in for a capture session that keeps running without touching any hardware."""
    await rosys.sleep(60.0)
    return True


async def test_rtsp_device_keeps_one_capture_loop_across_concurrent_restarts(rosys_integration):
    task_name = f'capture {GOODCAM_MAC}'
    with connected_rtsp_stream():
        device = RtspDevice(GOODCAM_MAC, '192.168.0.5', substream=0, fps=5,
                            on_new_image_data=lambda array, timestamp: None)
        try:
            await forward_until(lambda: device.is_connected, message='expected the capture session to come up')
            await asyncio.gather(device.restart_gstreamer(), device.restart_gstreamer())
            await asyncio.sleep(0.05)
            assert len(live_capture_tasks(task_name)) == 1, \
                f'expected one capture loop, got {len(live_capture_tasks(task_name))}'
        finally:
            await device.shutdown()
            await cancel_leftover_loops(task_name)


async def test_usb_camera_keeps_one_capture_loop_across_concurrent_reconnects(rosys_integration):
    camera_id = 'usb_reconnect_race'
    task_name = f'usb capture {camera_id}'
    with patch.object(UsbDevice, '_run_capture_session', _stub_usb_session):
        camera = UsbCamera(id=camera_id, connect_after_init=False)
        try:
            await camera.connect()
            await asyncio.sleep(0.05)
            await asyncio.gather(camera.reconnect(), camera.reconnect())
            await asyncio.sleep(0.05)
            assert len(live_capture_tasks(task_name)) == 1, \
                f'expected one capture loop, got {len(live_capture_tasks(task_name))}'
        finally:
            await camera.disconnect()
            await cancel_leftover_loops(task_name)


async def test_rtsp_camera_does_not_restart_a_device_it_is_disconnecting(rosys_integration):
    task_name = f'capture {GOODCAM_MAC}'
    camera = RtspCamera(mac=GOODCAM_MAC, ip='192.168.0.5', connect_after_init=False)
    with connected_rtsp_stream():
        try:
            await camera.connect()
            await forward_until(lambda: camera.is_connected, message='expected the capture session to come up')
            await asyncio.gather(camera.disconnect(), camera.set_parameters({'fps': 7}))
            await asyncio.sleep(0.05)
            assert camera.device is None, 'expected the camera to end up disconnected'
            assert not live_capture_tasks(task_name), 'a capture loop outlived the disconnect'
        finally:
            await camera.disconnect()
            await cancel_leftover_loops(task_name)


async def test_simulated_camera_disconnect_stops_loop(rosys_integration):
    camera = SimulatedCamera(id='sim_stop', width=64, height=48, fps=10, reconnect_interval=0.2)
    await camera.connect()
    await forward(0.4)
    assert camera.images, 'expected simulated camera to produce images while connected'

    device = camera.device
    assert device is not None, 'expected simulated camera to have a device after connect'
    await camera.disconnect()
    assert camera.device is None, 'expected camera.disconnect() to clear camera.device reference'
    assert device.is_active is False, 'expected old device loop to stop when camera disconnects'

    image_count_after_disconnect = len(camera.images)
    for _ in range(6):
        await forward(0.3)
    assert len(camera.images) == image_count_after_disconnect, 'expected no new images after camera disconnect'


def test_simulate_device_failure_deprecated_alias(rosys_integration):
    provider = SimulatedCameraProvider(auto_scan=False, simulate_failing=True)

    with pytest.warns(DeprecationWarning):
        deprecated_value = provider.simulate_device_failure
    assert deprecated_value is True, 'expected deprecated alias getter to proxy simulate_failing=True'

    with pytest.warns(DeprecationWarning):
        provider.simulate_device_failure = False
    assert provider.simulate_failing is False, 'expected deprecated alias setter to update simulate_failing'


async def test_devices_do_not_leak_shutdown_handlers(rosys_integration):
    handlers_before_device = len(rosys_core.shutdown_handlers)
    device = SimulatedDevice(id='sim_device_no_shutdown_hook',
                             size=ImageSize(width=64, height=48),
                             on_new_image=lambda image: None,
                             fps=10)
    try:
        assert len(rosys_core.shutdown_handlers) == handlers_before_device, (
            'expected direct device construction to avoid registering shutdown handlers'
        )
    finally:
        device.shutdown()

    handlers_before_camera = len(rosys_core.shutdown_handlers)
    camera = SimulatedCamera(id='sim_camera_shutdown_hook', connect_after_init=False)
    assert len(rosys_core.shutdown_handlers) == handlers_before_camera + 1, (
        'expected exactly one shutdown handler to be added for a camera instance'
    )
    assert rosys_core.shutdown_handlers[-1].__qualname__ == 'SimulatedCamera.disconnect', (
        'expected the camera to register its own disconnect as the shutdown handler'
    )

    camera_ref = weakref.ref(camera)
    del camera
    gc.collect()
    assert camera_ref() is None, 'expected the shutdown handler not to keep the camera alive'
    assert len(rosys_core.shutdown_handlers) == handlers_before_camera, (
        'expected the handler of a discarded camera to be unregistered'
    )

    cycling_camera = SimulatedCamera(id='sim_camera_cycle', fps=10, reconnect_interval=0.2, connect_after_init=False)
    await cycling_camera.connect()
    await forward(0.3)
    handlers_before_cycles = len(rosys_core.shutdown_handlers)

    for _ in range(3):
        await cycling_camera.disconnect()
        await forward(0.1)
        await cycling_camera.connect()
        await forward(0.3)

    assert len(rosys_core.shutdown_handlers) == handlers_before_cycles, (
        'expected repeated connect/disconnect cycles to avoid adding more shutdown handlers'
    )
    await cycling_camera.disconnect()


async def test_is_active_tracks_connect_and_disconnect(rosys_integration):
    camera = SimulatedCamera(id='sim', connect_after_init=False)
    assert camera.is_active is False
    await camera.connect()
    assert camera.is_active is True
    await camera.disconnect()
    assert camera.is_active is False


async def test_axis_device_derives_url_from_its_settings(rosys_integration):
    with stalled_mjpeg_stream():
        device = AxisMjpegDevice(AXIS_MAC, '192.168.0.5', index=1,
                                 on_new_image_data=lambda data, timestamp: None)
        try:
            assert device.url is not None
            assert 'camera=1' in device.url
            assert 'fps=6' in device.url and 'resolution=640x480' in device.url and 'mirror=0' in device.url

            await device.set_fps(12)
            await device.set_resolution(1280, 720)
            await device.set_mirrored(True)
            assert device.url is not None
            assert 'fps=12' in device.url and 'resolution=1280x720' in device.url and 'mirror=1' in device.url
            assert device.url.count('fps=') == 1, 'expected the URL to be rebuilt rather than appended to'
        finally:
            device.shutdown()


async def test_axis_device_url_follows_address_and_keeps_settings(rosys_integration):
    with stalled_mjpeg_stream():
        device = AxisMjpegDevice(AXIS_MAC, on_new_image_data=lambda data, timestamp: None)
        try:
            assert device.url is None, 'expected no URL while the address is unknown'
            assert device.is_active, 'expected the capture loop to run even without an address'

            await device.set_fps(12)
            device.ip = '192.168.0.5'
            assert device.url is not None and '192.168.0.5' in device.url
            assert 'fps=12' in device.url, 'expected the stream settings to survive the address change'
            assert device.is_active
        finally:
            device.shutdown()


async def test_mjpeg_camera_is_active_without_known_address(rosys_integration):
    """A camera that cannot reach its device yet still reports that a connection is wanted."""
    camera = MjpegCamera(id=GOODCAM_MAC, connect_after_init=False)
    await camera.connect()
    try:
        assert camera.device is not None, 'expected connect() to create a device even without an address'
        assert camera.device.url is None, 'expected no stream URL while the address is unknown'
        assert camera.is_active, 'expected the camera to report that it wants to be connected'
        assert not camera.is_connected
        assert camera.is_reconnecting
    finally:
        await camera.disconnect()


async def test_mjpeg_camera_stays_active_while_its_login_is_rejected(rosys_integration):
    """A rejected login does not revoke the wish to be connected, so `connect()` stays a no-op."""
    server = Unauthorized401Server()
    await server.start()
    camera = MjpegCamera(id=GOODCAM_MAC, ip=f'127.0.0.1:{server.port}',
                         connect_after_init=False, reconnect_interval=0.2)
    await camera.connect()
    device = camera.device
    assert device is not None
    try:
        await forward_until(lambda: not device.authorized, step=0.2,
                            message='expected the camera to be rejected by the 401 server')
        assert camera.is_active, 'expected a rejected camera to still report that it wants to be connected'
        assert not camera.is_connected
        assert camera.is_reconnecting
    finally:
        await camera.disconnect()
    assert not camera.is_active, 'expected only disconnect() to end the wish to be connected'


async def test_rtsp_camera_is_active_without_known_address(rosys_integration):
    camera = RtspCamera(mac=GOODCAM_MAC, connect_after_init=False)
    await camera.connect()
    try:
        assert camera.device is not None, 'expected connect() to create a device even without an address'
        assert camera.url is None, 'expected no stream URL while the address is unknown'
        assert camera.is_active, 'expected the camera to report that it wants to be connected'
        assert not camera.is_connected
    finally:
        await camera.disconnect()


async def test_usb_camera_is_active_without_video_device(rosys_integration):
    with patch('rosys.vision.usb_camera.usb_device.find_device_node', return_value=None):
        camera = UsbCamera(id='fakecam', connect_after_init=False, reconnect_interval=0.3)
        await camera.connect()
        try:
            await forward(0.5)
            assert camera.is_active, 'expected the camera to keep retrying while no video device exists'
            assert not camera.is_connected
        finally:
            await camera.disconnect()


async def test_mjpeg_provider_adds_and_connects_new_camera(rosys_integration):
    with stalled_mjpeg_stream():
        provider = MjpegCameraProvider(auto_scan=False)
        with patch.object(provider, 'scan_for_cameras', AsyncMock(return_value=[(GOODCAM_MAC, '127.0.0.1:1')])):
            await provider.update_device_list()
        camera = provider.cameras[GOODCAM_MAC]
        await asyncio.sleep(0.05)  # let the camera's own connect task run
        assert camera.is_active, 'expected a discovered camera to connect on its own'
        assert camera.ip == '127.0.0.1:1'
        await provider.shutdown()


async def test_mjpeg_provider_rebinds_moved_camera(rosys_integration):
    with stalled_mjpeg_stream():
        provider = MjpegCameraProvider(auto_scan=False)
        camera = MjpegCamera(id=GOODCAM_MAC, ip='127.0.0.1:1', connect_after_init=False)
        provider.add_camera(camera)
        await camera.connect()
        assert camera.is_active
        old_device = camera.device

        with patch.object(provider, 'scan_for_cameras', AsyncMock(return_value=[(GOODCAM_MAC, '127.0.0.1:2')])):
            await provider.update_device_list()

        assert camera.ip == '127.0.0.1:2'
        assert camera.device is old_device, 'expected the device to be rebound rather than torn down'
        assert camera.device is not None and '127.0.0.1:2' in (camera.device.url or '')
        assert camera.is_active
        await provider.shutdown()


async def test_mjpeg_provider_supplies_ip_to_pending_camera(rosys_integration):
    with stalled_mjpeg_stream():
        provider = MjpegCameraProvider(auto_scan=False)
        camera = MjpegCamera(id=GOODCAM_MAC, connect_after_init=False)
        provider.add_camera(camera)
        await camera.connect()  # no IP known yet, so the device has nothing to open
        assert camera.is_active
        assert camera.device is not None and camera.device.url is None

        with patch.object(provider, 'scan_for_cameras', AsyncMock(return_value=[(GOODCAM_MAC, '127.0.0.1:1')])):
            await provider.update_device_list()

        assert camera.ip == '127.0.0.1:1'
        assert camera.device is not None and '127.0.0.1:1' in (camera.device.url or ''), \
            'expected the device to pick up the discovered address'
        assert camera.is_active
        await provider.shutdown()


async def test_mjpeg_provider_leaves_disconnected_camera_alone(rosys_integration):
    with stalled_mjpeg_stream():
        provider = MjpegCameraProvider(auto_scan=False)
        camera = MjpegCamera(id=GOODCAM_MAC, ip='127.0.0.1:1', connect_after_init=False)
        provider.add_camera(camera)
        await camera.connect()
        await camera.disconnect()

        with patch.object(provider, 'scan_for_cameras', AsyncMock(return_value=[(GOODCAM_MAC, '127.0.0.1:2')])):
            await provider.update_device_list()

        assert camera.ip == '127.0.0.1:2', 'expected the provider to record the new address'
        assert not camera.is_active, 'expected the provider to leave a deliberately disconnected camera alone'


async def test_rtsp_provider_rebinds_moved_camera(rosys_integration):
    with stalled_rtsp_stream():
        provider = RtspCameraProvider(auto_scan=False)
        camera = RtspCamera(mac=GOODCAM_MAC, ip='192.168.0.5', connect_after_init=False)
        provider.add_camera(camera)
        await camera.connect()
        assert camera.is_active
        old_device = camera.device

        with patch('rosys.vision.rtsp_camera.rtsp_camera_provider.find_known_cameras',
                   AsyncMock(return_value=[(GOODCAM_MAC, '192.168.0.6')])):
            await provider.update_device_list()

        assert camera.ip == '192.168.0.6'
        assert camera.device is old_device, 'expected the device to be rebound rather than torn down'
        assert camera.device is not None and '192.168.0.6' in (camera.device.url or '')
        assert camera.is_active
        await provider.shutdown()


async def test_usb_camera_connects_once_video_device_appears(rosys_integration):
    device_node: str | None = None

    with patch('rosys.vision.usb_camera.usb_device.find_device_node', lambda _uid: device_node), \
            patch.object(UsbDevice, 'create_capture', lambda _device_node: FakeCapture()):
        camera = UsbCamera(id='fakecam', connect_after_init=False, reconnect_interval=0.3)
        await camera.connect()  # no video device node available yet
        try:
            await forward(0.5)
            assert camera.is_active and not camera.is_connected

            device_node = '/dev/video0'  # the node shows up, e.g. after the cable was plugged back in
            await forward_until(lambda: camera.is_connected, real_step=0.02,
                                message='expected the device to connect on its own once the node appeared')
        finally:
            await camera.disconnect()


async def test_usb_provider_adds_camera_for_new_device(rosys_integration):
    provider = UsbCameraProvider(auto_scan=False)
    with patch.object(UsbCameraProvider, 'scan_for_cameras', AsyncMock(return_value={'fakecam'})), \
            patch('rosys.vision.usb_camera.usb_device.find_device_node', return_value='/dev/video0'), \
            patch.object(UsbDevice, 'create_capture', lambda _device_node: FakeCapture()):
        await provider.update_device_list()
        camera = provider.cameras['fakecam']
        await asyncio.sleep(0.05)  # let the camera's own connect task run
        assert camera.is_active, 'expected a discovered camera to connect on its own'
        await provider.shutdown()


async def test_usb_provider_leaves_disconnected_camera_alone(rosys_integration):
    with patch.object(UsbCameraProvider, 'scan_for_cameras', AsyncMock(return_value={'fakecam'})), \
            patch('rosys.vision.usb_camera.usb_device.find_device_node', return_value='/dev/video0'), \
            patch.object(UsbDevice, 'create_capture', lambda _device_node: FakeCapture()):
        provider = UsbCameraProvider(auto_scan=False)
        camera = UsbCamera(id='fakecam', connect_after_init=False)
        provider.add_camera(camera)
        await camera.connect()
        assert camera.is_active
        await camera.disconnect()

        await provider.update_device_list()
        assert not camera.is_active, 'expected the provider to leave a deliberately disconnected camera alone'


async def test_simulated_provider_leaves_disconnected_camera_alone(rosys_integration):
    provider = SimulatedCameraProvider(auto_scan=False)
    camera = SimulatedCamera(id='sim', connect_after_init=False)
    provider.add_camera(camera)
    await camera.connect()
    await camera.disconnect()

    await provider.update_device_list()
    assert not camera.is_active, 'expected the provider to leave a deliberately disconnected camera alone'
