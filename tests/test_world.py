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
    data = json.loads(runtime.world.json())
    assert 'robot' in data.keys()
    assert 'automation_state' in data.keys()
    assert 'obstacles' in data.keys()
    assert 'notifications' in data.keys()
