import asyncio
from unittest.mock import patch

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


async def test_reconnect_task_running_after_initial_connect(rosys_integration):
    camera = ReconnectingSimulatedCamera(id='test_cam', width=800, height=600, fps=1, connect_after_init=True)
    await forward(seconds=1.0)
    assert camera.is_activated
    assert camera.is_connected


async def test_reconnecting_camera_stops_on_disconnect(rosys_integration):
    camera = ReconnectingSimulatedCamera(id='test_cam', width=800, height=600, fps=1, connect_after_init=False)
    await camera.connect()
    assert camera.is_activated

    await camera.disconnect()
    assert not camera.is_activated
    assert not camera.is_connected

    await forward(1.0)
    assert not camera.is_activated
    assert not camera.is_connected


def test_reconnecting_camera_persists_reconnect_interval(rosys_integration):
    camera = ReconnectingSimulatedCamera(id='test_cam', width=800, height=600, fps=1, reconnect_interval=999.0)
    camera_as_dict = camera.to_dict()
    assert camera_as_dict['reconnect_interval'] == 999.0

    restored_camera = ReconnectingSimulatedCamera.from_dict(camera_as_dict)
    assert restored_camera.reconnect_interval == 999.0


async def test_disconnect_during_reconnect_race(rosys_integration):
    """disconnect() called while _try_reconnect is mid-execution must not leave camera connected without reconnect task."""
    camera = ReconnectingSimulatedCamera(id='test_cam', width=800, height=600, fps=1, reconnect_interval=0.1)
    await camera.connect()
    camera.device = None
    assert not camera.is_connected

    reconnect_connect_released = asyncio.Event()
    reconnect_connect_reached = asyncio.Event()
    original_connect = SimulatedCamera.connect

    async def blocking_connect(self):
        reconnect_connect_reached.set()
        await reconnect_connect_released.wait()
        return await original_connect(self)

    with patch.object(SimulatedCamera, 'connect', blocking_connect):
        await forward(0.2)
        await asyncio.wait_for(reconnect_connect_reached.wait(), timeout=2.0)
        # disconnect_task is created before releasing the reconnect, so it lands earlier
        # in asyncio's ready queue. Without the _device_connection mutex in _try_reconnect,
        # disconnect() would run first, clear _reconnect_repeater and disconnect, then
        # _try_reconnect would resume and reconnect — leaving the camera in a bad state.
        disconnect_task = asyncio.create_task(camera.disconnect())
        reconnect_connect_released.set()
        await disconnect_task

    assert not camera.is_connected
    assert not camera.is_activated
