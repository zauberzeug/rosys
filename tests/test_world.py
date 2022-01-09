import json
from rosys import Persistence
from rosys.test import TestRuntime


def test_dumping(runtime: TestRuntime):
    data = Persistence(runtime.world).dump()
    assert 'robot' in data.keys()
    assert 'obstacles' in data.keys()
    assert 'automation_state' not in data.keys()


def test_json_view(runtime: TestRuntime):
    '''exporting whole world as json is needed for example for logging/inspection'''
    serialized = runtime.world.json()
    data = json.loads(serialized)
    assert 'robot' in data.keys()
    assert 'automation_state' in data.keys()
    assert 'obstacles' in data.keys()
    assert 'notifications' in data.keys()
    assert 'cameras' in data.keys()
    assert 'frames' not in data['cameras']['simulated_cam_0'].keys()
