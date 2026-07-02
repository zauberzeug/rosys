import asyncio
from unittest.mock import patch

import numpy as np
import pytest

import rosys
import rosys.rosys as rosys_core
from rosys.testing import forward
from rosys.vision import ImageSize, MjpegCamera, RtspCamera, SimulatedCamera, SimulatedCameraProvider, UsbCamera
from rosys.vision.mjpeg_camera.mjpeg_device import MjpegDevice
from rosys.vision.rtsp_camera.rtsp_device import RtspDevice
from rosys.vision.simulated_camera.simulated_device import SimulatedDevice
from rosys.vision.usb_camera.usb_device import UsbDevice

# A MAC that maps to a "GOODCAM" URL in both the RTSP and MJPEG vendor tables.
# GOODCAM needs no settings interface, so the device is constructed without any network access.
GOODCAM_MAC = '2c:6f:51:00:00:01'


class FlakyMjpegServer:
    """Local HTTP server that serves a single fake JPEG frame per connection and then drops it.

    Each connection mimics a camera whose stream ends, so a device pointed at it must reconnect
    to receive more frames.
    """

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
            writer.write(b'HTTP/1.1 200 OK\r\n'
                         b'Content-Type: multipart/x-mixed-replace; boundary=frame\r\n'
                         b'Connection: close\r\n\r\n')
            writer.write(b'\xff\xd8' + bytes(32) + b'\xff\xd9')  # minimal SOI..EOI JPEG marker pair
            await writer.drain()
        finally:
            writer.close()

    async def stop(self) -> None:
        assert self._server is not None
        self._server.close()
        await self._server.wait_closed()


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
        for _ in range(60):
            await forward(0.3)  # advance simulated time past the reconnect interval
            await asyncio.sleep(0.05)  # give the real loopback I/O a chance to run
            if server.connections >= 3 and len(frames) >= 3:
                break
        assert server.connections >= 3, f'device did not reconnect (only {server.connections} connections)'
        assert len(frames) >= 3, f'no frames received across reconnects (got {len(frames)})'

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
    async def no_stream(self) -> None:
        await rosys.sleep(7.0)

    camera = RtspCamera(mac=GOODCAM_MAC, ip='192.168.0.5', reconnect_interval=7.0, connect_after_init=False)
    with patch.object(RtspDevice, '_run_gstreamer', no_stream):
        await camera.connect()
        assert camera.device is not None
        assert camera.device.reconnect_interval == 7.0
        await camera.disconnect()


async def test_mjpeg_camera_passes_reconnect_interval_to_device(rosys_integration):
    async def no_stream(self) -> None:
        await rosys.sleep(7.0)

    camera = MjpegCamera(id=GOODCAM_MAC, ip='127.0.0.1:1', reconnect_interval=7.0, connect_after_init=False)
    with patch.object(MjpegDevice, 'run_capture_task', no_stream):
        await camera.connect()
        assert camera.device is not None
        assert camera.device.reconnect_interval == 7.0
        await camera.disconnect()


class FakeCapture:
    """Minimal stand-in for cv2.VideoCapture that can simulate a disconnected device."""

    def __init__(self) -> None:
        self._opened = True
        self._frame = np.zeros((48, 64, 3), dtype=np.uint8)

    def isOpened(self) -> bool:  # cv2 API name
        return self._opened

    def read(self):
        if not self._opened:
            return False, None
        return True, self._frame

    def release(self) -> None:
        self._opened = False

    def get(self, _prop) -> float:
        return 0.0

    def set(self, _prop, _value) -> bool:
        return True


async def test_usb_device_reconnects_after_disconnect(rosys_integration):
    captures: list[FakeCapture] = []

    def make_capture(_index: int) -> FakeCapture:
        capture = FakeCapture()
        captures.append(capture)
        return capture

    frames: list = []
    with patch.object(UsbDevice, 'create_capture', make_capture), \
            patch('rosys.vision.usb_camera.usb_device.find_video_id', return_value=0):
        device = UsbDevice.from_uid('fakecam', lambda data, timestamp: frames.append(data), reconnect_interval=0.3)
        assert device is not None
        try:
            for _ in range(15):
                await forward(0.1)
                await asyncio.sleep(0.02)
                if frames:
                    break
            assert frames, 'device did not capture any frames'
            assert device.is_connected
            captures_before = len(captures)

            captures[-1].release()  # simulate a bad cable: the capture dies
            for _ in range(20):
                await forward(0.3)
                await asyncio.sleep(0.02)
                if len(captures) > captures_before:
                    break
            assert len(captures) > captures_before, 'device did not reopen its capture'
            assert device.is_connected

            frames_after_reconnect = len(frames)
            for _ in range(10):
                await forward(0.1)
                await asyncio.sleep(0.02)
                if len(frames) > frames_after_reconnect:
                    break
            assert len(frames) > frames_after_reconnect, 'frames did not resume after reconnect'

            await device.shutdown()
            assert not device.is_connected
            captures_at_shutdown = len(captures)
            for _ in range(5):
                await forward(0.3)
                await asyncio.sleep(0.02)
            assert len(captures) == captures_at_shutdown, 'device kept reopening after shutdown'
        finally:
            await device.shutdown()


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


async def test_mjpeg_device_gives_up_on_401(rosys_integration):
    server = Unauthorized401Server()
    await server.start()
    device = MjpegDevice(GOODCAM_MAC, f'127.0.0.1:{server.port}',
                         on_new_image_data=lambda data, timestamp: None,
                         reconnect_interval=0.2)
    try:
        for _ in range(20):
            await forward(0.2)
            await asyncio.sleep(0.05)
            if server.connections >= 1 and not device.is_active:
                break
        assert server.connections >= 1, f'expected at least one connection attempt, got {server.connections}'
        assert device.authorized is False, 'expected device to mark itself unauthorized after 401 response'
        assert device.is_active is False, 'expected reconnect loop to stop after unauthorized response'
        assert device.is_connected is False, 'expected unauthorized device to remain disconnected'

        connections_after_give_up = server.connections
        for _ in range(10):
            await forward(0.3)
            await asyncio.sleep(0.05)
        assert server.connections == connections_after_give_up, (
            'expected no further reconnect attempts after 401 unauthorized response'
        )
    finally:
        device.shutdown()
        await server.stop()


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
    latest_handler = rosys_core.shutdown_handlers[-1]
    assert getattr(latest_handler, '__self__', None) is camera, (
        'expected camera shutdown handler to be bound to that camera instance'
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
