from robot import Robot, Drive
import asyncio
import pytest
from tests.test_helper import Robot
import socketio


@pytest.mark.asyncio
async def test_linear_forward(sio: socketio.Client, robot: Robot):
    robot.assert_pose(0, 0)
    sio.call('drive_power', Drive(left=1, right=1).dict())
    sio.call('fast_forward', 10)
    robot.assert_pose(0, 10)
