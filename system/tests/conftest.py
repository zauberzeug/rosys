import pytest
from typing import Generator
import socketio
import asyncio
import functools
import requests
import tests.test_helper as test_helper


@pytest.fixture
def web() -> Generator:
    with test_helper.LiveServerSession() as c:
        yield c


@pytest.fixture
async def sio(event_loop) -> Generator:

    sio = socketio.Client()
    is_connected = asyncio.Event()

    @sio.event
    def connect():
        event_loop.call_soon_threadsafe(is_connected.set())

    sio.connect("ws://localhost", socketio_path="/ws/socket.io", transports=['websocket'])

    await asyncio.wait_for(is_connected.wait(), 1)
    assert sio.transport() == 'websocket'

    yield sio
    sio.disconnect()


@pytest.fixture
def robot(sio) -> Generator:
    r = test_helper.Robot()
    print('..xxxxxxxx...', flush=True)

    @sio.on('robot_pose')
    def robot_pose(data):
        robot = test_helper.Robot.parse_obj(data)
        print('.....', str(data), robot, flush=True)

    yield r
