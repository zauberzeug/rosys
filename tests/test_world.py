import json

import pytest
from rosys import Persistence


def test_dumping():
    data = Persistence(runtime.world).dump()
    assert 'areas' in data
    assert 'obstacles' in data
    assert 'usb_cameras' in data


@pytest.mark.asyncio
async def test_json_view():
    '''exporting whole world as json is needed for example for logging/inspection'''
    runtime.with_usb_cameras()
    await runtime.forward(1)
    await runtime.world.add_usb_camera(runtime.usb_camera_simulator.create_calibrated('simulated_cam_0'))
    serialized = runtime.world.json()
    data = json.loads(serialized)
    assert 'robot' in data
    assert 'obstacles' in data
    assert 'notifications' in data
    assert 'usb_cameras' in data
    assert 'images' not in data['usb_cameras']['simulated_cam_0']
