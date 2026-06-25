import asyncio
from unittest.mock import patch

import rosys
from rosys.testing import forward
from rosys.vision import MjpegCamera, RtspCamera
from rosys.vision.mjpeg_camera.mjpeg_device import MjpegDevice
from rosys.vision.rtsp_camera.rtsp_device import RtspDevice

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
        assert device.is_connected

        await device.shutdown()
        assert not device.is_connected
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
