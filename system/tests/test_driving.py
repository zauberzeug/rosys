from robot import Robot, Drive
import asyncio
import pytest
from tests.test_helper import Robot
import socketio


@pytest.mark.asyncio
async def test_linear_forward(sio: socketio.Client, robot: Robot):
    sio.call('drive_power', Drive(left=10, right=10).dict())
    sio.call('fast_forward', 10)
    robot.assert_pose(0, 100)
