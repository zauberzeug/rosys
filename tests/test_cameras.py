import asyncio
import gc
import platform

import pytest
from nicegui import app

from rosys.testing import forward
from rosys.vision import RtspCamera, RtspCameraProvider, SimulatedCamera, UsbCamera, UsbCameraProvider


async def test_simulated_camera(rosys_integration):
    camera = SimulatedCamera(id='test_cam', resolution=(800, 600), fps=1)
    await camera.connect()
    assert camera.is_connected
    await forward(1.1)
    assert len(camera.images) >= 1
    assert camera.images[0].size.width == 800
    assert camera.images[0].size.height == 600


async def test_usb_camera(rosys_integration):
    if platform.system() != 'Linux':
        pytest.skip('UsbCamera is only supported on Linux.')
    connected_uids = list(await UsbCameraProvider.scan_for_cameras())
    if len(connected_uids) == 0:
        pytest.skip('No USB camera detected. This test requires a physical USB camera to be connected.')
    camera = UsbCamera(id=connected_uids[0])
    await camera.connect()
    for _ in range(5):
        await forward(0.1)
        await asyncio.sleep(0.2)
    assert camera.is_connected
    assert len(camera.images) >= 1


async def test_rtsp_camera(rosys_integration):
    try:
        connected_uids = await RtspCameraProvider.scan_for_cameras()
    except Exception as e:
        if 'Could not run arp-scan!' in str(e):
            pytest.skip('arp-scan is not installed. This test requires arp-scan to be installed.')
        raise
    if len(connected_uids) == 0:
        pytest.skip('No RTSP camera detected. This test requires a physical RTSP camera on the local network.')
    mac, ip = connected_uids[0]
    camera = RtspCamera(mac=mac, ip=ip)
    await camera.connect()
    for _ in range(5):
        await forward(0.1)
        await asyncio.sleep(0.2)
    assert camera.is_connected
    assert len(camera.images) >= 1


def _image_route_count(camera_id: str) -> int:
    return len([route for route in app.routes if getattr(route, 'path', '').startswith(f'/images/{camera_id}/')])


async def test_image_routes_are_released_with_the_camera(rosys_integration):
    camera = SimulatedCamera(id='fleeting_cam', connect_after_init=False)
    assert _image_route_count('fleeting_cam') == 3

    del camera
    gc.collect()

    assert _image_route_count('fleeting_cam') == 0


async def test_a_collected_camera_keeps_the_routes_of_its_successor(rosys_integration):
    old_camera = SimulatedCamera(id='reused_cam', connect_after_init=False)
    new_camera = SimulatedCamera(id='reused_cam', connect_after_init=False)  # same id: replaces the routes

    del old_camera
    gc.collect()

    assert _image_route_count('reused_cam') == 3
    assert new_camera.base_path == 'images/reused_cam'


async def test_a_pending_finalizer_cannot_remove_the_routes_of_a_reused_id(rosys_integration, monkeypatch):
    """The old camera's finalizer may fire while its successor is still registering its routes."""
    old_camera: SimulatedCamera | None = SimulatedCamera(id='raced_cam', connect_after_init=False)
    original_add_api_route = app.add_api_route

    def add_api_route(*args, **kwargs):
        nonlocal old_camera
        result = original_add_api_route(*args, **kwargs)
        old_camera = None  # NOTE: collect the old camera once the successor has registered its first route
        gc.collect()
        return result
    monkeypatch.setattr(app, 'add_api_route', add_api_route)

    new_camera = SimulatedCamera(id='raced_cam', connect_after_init=False)

    assert _image_route_count('raced_cam') == 3
    assert new_camera.base_path == 'images/raced_cam'
