import asyncio
import logging
from typing import Any, Callable, Optional, Union

import numpy as np
import pytest

from .. import run
from ..actors import Automator, Driver, Odometer, PathPlanner
from ..hardware import Wheels
from ..runtime import runtime
from ..world import Point, Point3d

log = logging.getLogger(__name__)

automator: Optional[Automator] = None
driver: Optional[Driver] = None
odometer: Optional[Odometer] = None
path_planner: Optional[PathPlanner] = None
wheels: Optional[Wheels] = None


async def forward(seconds: Optional[float] = None,
                  *,
                  until: Optional[Union[int, float, Callable]] = None,
                  x: Optional[float] = None,
                  y: Optional[float] = None,
                  tolerance: float = 0.1,
                  dt: float = 0.01,
                  timeout: float = 100):
    start_time = runtime.time
    if seconds is not None:
        msg = f'forwarding {seconds=}'
        def condition(): return runtime.time >= start_time + seconds
        timeout = max(timeout, seconds)
    elif isinstance(until, int) or isinstance(until, float):
        msg = f'forwarding {until=}'
        def condition(): return runtime.time >= until
        timeout = max(timeout, until - start_time)
    elif isinstance(until, Callable):
        msg = f'forwarding {until=}'
        condition = until
    elif x is not None and y is not None:
        assert odometer is not None
        msg = f'forwarding to {x=} and {y=}'
        def condition(): return (odometer.prediction.x - x)**2 + (odometer.prediction.y - y)**2 < tolerance**2
    elif x is not None:
        assert odometer is not None
        msg = f'forwarding to {x=}'
        def condition(): return abs(odometer.prediction.x - x) < tolerance
    elif y is not None:
        assert odometer is not None
        msg = f'forwarding to {y=}'
        def condition(): return abs(odometer.prediction.y - y) < tolerance
    else:
        raise Exception('invalid arguments')

    log.info(f'\033[94m{msg}\033[0m')
    while not condition():
        if runtime.time > start_time + timeout:
            raise TimeoutError(f'condition took more than {timeout} s')
        if not run.running_cpu_bound_processes:
            runtime.set_time(runtime.time + dt)
            await asyncio.sleep(0)
        else:
            await asyncio.sleep(0.01)
        if runtime._exception is not None:
            raise RuntimeError(f'error while forwarding time {dt} s') from runtime._exception


def assert_pose(x: float, y: float, *, deg: float = None, linear_tolerance: float = 0.1, deg_tolerance: float = 1.0) -> None:
    assert odometer is not None
    assert odometer.prediction.x == pytest.approx(x, abs=linear_tolerance)
    assert odometer.prediction.y == pytest.approx(y, abs=linear_tolerance)
    if deg is not None:
        assert np.rad2deg(odometer.prediction.yaw) == pytest.approx(deg, abs=deg_tolerance)


def assert_point(actual: Union[Point, Point3d], expected: Union[Point, Point3d], tolerance=0.1) -> None:
    assert type(actual) == type(expected)
    assert actual.x == pytest.approx(expected.x, abs=tolerance)
    assert actual.y == pytest.approx(expected.y, abs=tolerance)
    if type(actual) is Point3d:
        assert actual.z == pytest.approx(expected.z, abs=tolerance)


async def automate_drive_to(x: float, y: float) -> None:
    assert automator is not None
    assert driver is not None
    automator.start(driver.drive_to(Point(x=x, y=y)))


def approx(o1: Any, o2: Any) -> None:
    # https://github.com/pytest-dev/pytest/issues/6632#issuecomment-580507745
    assert type(o1) == type(o2)

    o1_keys = [v for v in dir(o1) if not v.startswith('__')]
    o2_keys = [v for v in dir(o2) if not v.startswith('__')]

    assert sorted(o1_keys) == sorted(o2_keys)

    for k in o1_keys:
        v1 = getattr(o1, k)
        v2 = getattr(o2, k)
        if isinstance(v1, int) or isinstance(v1, float):
            assert v1 == pytest.approx(v2)
            continue

        if isinstance(v1, bool) or isinstance(v1, str):
            assert v1 == v2
            continue

        approx(v1, v2)
