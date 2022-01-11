import json
import pytest
from rosys import Persistence
from rosys.test import TestRuntime


def test_dumping(runtime: TestRuntime):
    data = Persistence(runtime.world).dump()
    assert 'robot' in data
    assert 'obstacles' in data
    assert 'automation_state' not in data


@pytest.mark.asyncio
async def test_json_view(runtime: TestRuntime):
    '''exporting whole world as json is needed for example for logging/inspection'''
    await runtime.forward(1)
    serialized = runtime.world.json()
    data = json.loads(serialized)
    assert 'robot' in data
    assert 'automation_state' in data
    assert 'obstacles' in data
    assert 'notifications' in data
    assert 'cameras' in data
    assert 'frames' not in data['cameras']['simulated_cam_0']
