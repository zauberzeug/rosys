from rosys.testing import forward
from rosys.vision import ReconnectingCamera, SimulatedCamera


class ReconnectingSimulatedCamera(ReconnectingCamera, SimulatedCamera):
    """Simulated camera with automatic reconnection for testing."""


async def test_reconnecting_camera_connects(rosys_integration):
    camera = ReconnectingSimulatedCamera(id='test_cam', width=800, height=600, fps=1)
    await camera.connect()
    assert camera.is_connected
    assert camera.is_activated


async def test_reconnecting_camera_reconnects_after_disconnect(rosys_integration):
    camera = ReconnectingSimulatedCamera(id='test_cam', width=800, height=600, fps=1, reconnect_interval=0.5)
    await camera.connect()
    assert camera.is_connected

    # Simulate connection loss by clearing the device
    camera.device = None
    assert not camera.is_connected

    # Wait for reconnection
    await forward(1.0)
    assert camera.is_connected


async def test_reconnecting_camera_stops_on_disconnect(rosys_integration):
    camera = ReconnectingSimulatedCamera(id='test_cam', width=800, height=600, fps=1)
    await camera.connect()
    assert camera.is_activated

    await camera.disconnect()
    assert not camera.is_activated
    assert not camera.is_connected

    await forward(1.0)
    assert not camera.is_activated
    assert not camera.is_connected
