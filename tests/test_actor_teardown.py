import rosys.rosys as core
from rosys.driving import Odometer, Steerer
from rosys.hardware import RobotSimulation, WheelsSimulation
from rosys.testing import forward


def _live_repeaters() -> int:
    return len([task for task in core.Repeater.tasks if not task.done()])


async def test_actors_can_be_torn_down(rosys_integration: None) -> None:
    """tear_down() stops the per-instance repeaters so the instance can be released."""
    baseline = _live_repeaters()

    wheels = WheelsSimulation()
    robot = RobotSimulation([wheels])
    steerer = Steerer(wheels)
    odometer = Odometer(wheels)
    await forward(seconds=0.1)
    assert _live_repeaters() >= baseline + 3  # RobotSimulation, Steerer, Odometer each add one

    for actor in (robot, steerer, odometer):
        actor.tear_down()
    await forward(seconds=0.1)
    assert _live_repeaters() == baseline
