from rosys.driving import Steerer
from rosys.driving.steerer import State
from rosys.hardware import Wheels
from rosys.testing import forward


async def test_stop_resets_speeds(wheels: Wheels) -> None:
    steerer = Steerer(wheels)
    steerer.start()
    steerer.update(0.5, 1.0)
    await forward(seconds=0.1)
    assert (steerer.linear_speed, steerer.angular_speed) == (1.0, -0.5)
    assert (wheels.linear_target_speed, wheels.angular_target_speed) == (1.0, -0.5)

    steerer.stop()
    assert (steerer.linear_speed, steerer.angular_speed) == (0.0, 0.0)
    await forward(seconds=0.1)
    assert steerer.state == State.IDLE
    assert (wheels.linear_target_speed, wheels.angular_target_speed) == (0.0, 0.0)

    steerer.start()
    await forward(seconds=0.1)
    assert (wheels.linear_target_speed, wheels.angular_target_speed) == (0.0, 0.0)


async def test_immediate_restart_after_stop_does_not_keep_old_speed(wheels: Wheels) -> None:
    steerer = Steerer(wheels)
    steerer.start()
    steerer.update(0.5, 1.0)
    await forward(seconds=0.1)
    assert (wheels.linear_target_speed, wheels.angular_target_speed) == (1.0, -0.5)

    steerer.stop()
    steerer.start()
    await forward(seconds=0.1)
    assert (wheels.linear_target_speed, wheels.angular_target_speed) == (0.0, 0.0)
