from robot import Robot, Drive


def test_linear_forward(sio):
    sio.call('drive_power', Drive(left=10, right=2).dict())
