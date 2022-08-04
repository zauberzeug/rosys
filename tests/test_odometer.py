from rosys.driving import Odometer
from rosys.event import Event
from rosys.geometry import Pose, Velocity


class DummyVelocityProvider:
    VELOCITY_MEASURED = Event()


async def test_odometer():
    wheels = DummyVelocityProvider()
    odometer = Odometer(wheels)
    odometer.handle_velocities([Velocity(linear=0.0, angular=0.0, time=0.0)])
    assert odometer.prediction.x == 0.0

    for t in range(1, 11):
        odometer.handle_velocities([Velocity(linear=1.0, angular=0.0, time=t)])
    assert odometer.prediction.x == 10.0
    assert odometer.prediction.time == 10.0

    odometer.handle_detection(Pose(x=5.5, time=5.0))
    assert odometer.prediction.x == 10.5
    assert odometer.prediction.time == 10.0

    odometer.handle_detection(Pose(x=6.5, time=7.0))
    assert odometer.prediction.x == 9.5
    assert odometer.prediction.time == 10.0

    odometer.handle_detection(Pose(x=7.5, time=7.5))
    assert odometer.prediction.x == 10.0
    assert odometer.prediction.time == 10.0
