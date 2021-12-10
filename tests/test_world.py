from rosys.runtime import Runtime
from rosys.test import TestRuntime
from rosys import Persistence
import json


def test_dumping(runtime: TestRuntime):
    data = Persistence(runtime.world).dump()
    assert 'robot' in data.keys()
    assert 'obstacles' in data.keys()
    assert 'automation_state' not in data.keys()


def test_json_view(runtime: Runtime):
    data = json.loads(runtime.world.json())
    assert 'robot' in data.keys()
    assert 'automation_state' in data.keys()
    assert 'obstacles' in data.keys()
    assert 'notifications' in data.keys()
