import asyncio
import logging
from collections.abc import Callable
from dataclasses import fields, is_dataclass
from typing import Any

import numpy as np
import pytest

from .. import rosys, run
from ..automation import Automator
from ..driving import Driver, Odometer
from ..geometry import Point, Point3d, Pose3d

log = logging.getLogger(__name__)

automator: Automator | None = None
driver: Driver | None = None
odometer: Odometer | None = None


async def forward(seconds: float | None = None,
                  *,
                  until: int | None | float | Callable = None,
                  x: float | None = None,
                  y: float | None = None,
                  tolerance: float = 0.1,
                  dt: float = 0.01,
                  timeout: float = 100):
    start_time = rosys.time()
    if seconds is not None:
        def condition():
            return rosys.time() >= start_time + seconds
        msg = f'forwarding {seconds=}'
        timeout = max(timeout, seconds)
    elif isinstance(until, int | float):
        def condition():
            return rosys.time() >= until
        msg = f'forwarding {until=}'
        timeout = max(timeout, until - start_time)
    elif callable(until):
        msg = f'forwarding {until=}'
        condition = until
    elif x is not None and y is not None:
        def condition():
            assert odometer is not None
            return (odometer.prediction.x - x)**2 + (odometer.prediction.y - y)**2 < tolerance**2
        msg = f'forwarding to {x=} and {y=}'
    elif x is not None:
        def condition():
            assert odometer is not None
            return abs(odometer.prediction.x - x) < tolerance
        msg = f'forwarding to {x=}'
    elif y is not None:
        def condition():
            assert odometer is not None
            return abs(odometer.prediction.y - y) < tolerance
        msg = f'forwarding to {y=}'
    else:
        raise ValueError('invalid arguments')

    log.info('\033[94m%s\033[0m', msg)
    while not condition():
        if rosys.time() > start_time + timeout:
            raise TimeoutError(f'condition took more than {timeout} s')
        if not run.running_cpu_bound_processes:
            rosys.set_time(rosys.time() + dt)
            await asyncio.sleep(0)
        else:
            await asyncio.sleep(0.01)
        exception = rosys.get_last_exception()
        if exception is not None:
            raise RuntimeError(f'error while forwarding time {dt} s') from exception


def assert_pose(x: float, y: float, *, deg: float | None = None, position_tolerance: float = 0.1, deg_tolerance: float = 1.0) -> None:
    assert odometer is not None
    assert odometer.prediction.x == pytest.approx(x, abs=position_tolerance)
    assert odometer.prediction.y == pytest.approx(y, abs=position_tolerance)
    if deg is not None:
        assert np.rad2deg(odometer.prediction.yaw) == pytest.approx(deg, abs=deg_tolerance)


def assert_point(actual: Point | Point3d, expected: Point | Point3d, tolerance=0.1) -> None:
    assert type(actual) is type(expected)
    assert actual.x == pytest.approx(expected.x, abs=tolerance)
    assert actual.y == pytest.approx(expected.y, abs=tolerance)
    if isinstance(actual, Point3d) and isinstance(expected, Point3d):
        assert actual.z == pytest.approx(expected.z, abs=tolerance)


def poses_equal(pose1: Pose3d, pose2: Pose3d) -> bool:
    return np.allclose(pose1.translation, pose2.translation, atol=1e-6) and \
        np.allclose(pose1.rotation.matrix, pose2.rotation.matrix, atol=1e-6)


async def automate_drive_to(x: float, y: float) -> None:
    assert automator is not None
    assert driver is not None
    automator.start(driver.drive_to(Point(x=x, y=y)))


def approx(o1: Any, o2: Any, *,
           rel: float | None = None,
           abs: float | None = None,  # pylint: disable=redefined-builtin
           nan_ok: bool = False) -> None:
    if hasattr(o1, '__dict__'):
        assert sorted(o1.__dict__) == sorted(o2.__dict__)
        for key, value in o1.__dict__.items():
            approx(value, getattr(o2, key), rel=rel, abs=abs, nan_ok=nan_ok)
    elif is_dataclass(o1):
        o1_keys = [f.name for f in fields(o1)]
        o2_keys = [f.name for f in fields(o2)]
        assert sorted(o1_keys) == sorted(o2_keys)
        for key in o1_keys:
            approx(getattr(o1, key), getattr(o2, key), rel=rel, abs=abs, nan_ok=nan_ok)
    elif isinstance(o1, list):
        for i1, i2 in zip(o1, o2, strict=True):
            approx(i1, i2, rel=rel, abs=abs, nan_ok=nan_ok)
    elif isinstance(o1, dict):
        for k1, k2 in zip(sorted(o1), sorted(o2), strict=True):
            approx(o1[k1], o2[k2], rel=rel, abs=abs, nan_ok=nan_ok)
    else:
        assert o1 == pytest.approx(o2, rel=rel, abs=abs, nan_ok=nan_ok)
