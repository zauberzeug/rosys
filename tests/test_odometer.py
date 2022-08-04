import numpy as np
from rosys.driving import Odometer
from rosys.event import Event
from rosys.geometry import Pose, Velocity
from rosys.test import approx


class DummyVelocityProvider:
    VELOCITY_MEASURED = Event()


async def test_odometer():
    wheels = DummyVelocityProvider()
    odometer = Odometer(wheels)
    odometer.handle_velocities([Velocity(linear=0.0, angular=0.0, time=0.0)])
    assert odometer.prediction.x == 0.0

    for t in range(1, 11):
        odometer.handle_velocities([Velocity(linear=1.0, angular=0.0, time=t)])
    assert odometer.prediction == Pose(x=10.0, time=10.0)
    assert odometer.get_pose(8.5) == Pose(x=8.5, time=8.5)

    odometer.handle_detection(Pose(x=5.5, time=5.0))
    assert odometer.prediction == Pose(x=10.5, time=10.0)
    assert odometer.get_pose(8.5) == Pose(x=9.0, time=8.5)

    odometer.handle_detection(Pose(x=6.5, time=7.0))
    assert odometer.prediction == Pose(x=9.5, time=10.0)
    assert odometer.get_pose(8.5) == Pose(x=8.0, time=8.5)

    odometer.handle_detection(Pose(x=7.5, time=7.5))
    assert odometer.prediction == Pose(x=10.0, time=10.0)
    assert odometer.get_pose(8.5) == Pose(x=8.5, time=8.5)


def test_odometry_frame():
    local_pose = Pose(x=2.0, y=-0.5, yaw=np.deg2rad(45), time=1)
    detection = Pose(x=1.5, y=4.0, yaw=np.deg2rad(135), time=2)
    odometry_frame = Odometer._compute_odometry_frame(local_pose, detection)
    approx(odometry_frame, Pose(x=1.0, y=2.0, yaw=np.deg2rad(90), time=2))
