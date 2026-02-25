import asyncio
from unittest.mock import patch

from rosys.testing import forward
from rosys.vision import SimulatedCamera


async def test_camera_activates(rosys_integration):
    camera = SimulatedCamera(id='test_cam', width=800, height=600, fps=1)
    await camera.activate()
    assert camera.is_connected
    assert camera.is_reconnecting


async def test_camera_reconnects_after_connection_loss(rosys_integration):
    camera = SimulatedCamera(id='test_cam', width=800, height=600, fps=1, reconnect_interval=0.5)
    await camera.activate()
    assert camera.is_connected

    # Simulate connection loss by clearing the device
    camera.device = None
    assert not camera.is_connected

    # Wait for reconnection
    await forward(1.0)
    assert camera.is_connected


async def test_reconnect_repeater_running_after_init(rosys_integration):
    camera = SimulatedCamera(id='test_cam', width=800, height=600, fps=1, connect_after_init=False)
    await camera.activate()
    await forward(seconds=1.0)
    assert camera.is_reconnecting
    assert camera.is_connected


async def test_camera_stops_reconnecting_on_deactivate(rosys_integration):
    camera = SimulatedCamera(id='test_cam', width=800, height=600, fps=1, connect_after_init=False)
    await camera.activate()
    assert camera.is_reconnecting

    await camera.deactivate()
    assert not camera.is_reconnecting
    assert not camera.is_connected

    await forward(1.0)
    assert not camera.is_reconnecting
    assert not camera.is_connected


def test_camera_persists_reconnect_interval(rosys_integration):
    camera = SimulatedCamera(id='test_cam', width=800, height=600, fps=1, reconnect_interval=999.0)
    camera_as_dict = camera.to_dict()
    assert camera_as_dict['reconnect_interval'] == 999.0

    restored_camera = SimulatedCamera.from_dict(camera_as_dict)
    assert restored_camera.reconnect_interval == 999.0


async def test_deactivate_during_reconnect_race(rosys_integration):
    """deactivate() called while _try_reconnect is mid-execution must not leave camera connected without reconnect repeater."""
    camera = SimulatedCamera(id='test_cam', width=800, height=600, fps=1, reconnect_interval=0.1, connect_after_init=False)
    await camera.activate()
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
        # Trigger _try_reconnect directly as a task
        reconnect_task = asyncio.create_task(camera._try_reconnect())
        # Yield enough times for reconnect_task to reach blocking_connect
        for _ in range(20):
            if reconnect_connect_reached.is_set():
                break
            await asyncio.sleep(0)
        assert reconnect_connect_reached.is_set(), '_try_reconnect never reached connect()'
        # deactivate races with the in-progress reconnect; the mutex ensures clean teardown
        deactivate_task = asyncio.create_task(camera.deactivate())
        reconnect_connect_released.set()
        await asyncio.gather(reconnect_task, deactivate_task)

    assert not camera.is_connected
    assert not camera.is_reconnecting
