from robot import Robot, Drive
import asyncio
import pytest
from tests.test_helper import Robot
import socketio


@pytest.mark.asyncio
async def test_linear_forward(sio: socketio.Client, robot: Robot):
    sio.call('drive_power', Drive(left=10, right=2).dict())
    await asyncio.sleep(2)
    assert robot.pose.position.y == 10.0
