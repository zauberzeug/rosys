from typing import Literal

import pytest

import rosys
from rosys.automation import Automator
from rosys.driving import Driver
from rosys.geometry import Pose, Spline
from rosys.hardware import Robot
from rosys.testing import assert_pose, forward


async def test_pause_and_resume_spline(driver: Driver, automator: Automator, robot: Robot):
    spline = Spline.from_poses(Pose(x=0, y=0, yaw=0), Pose(x=2, y=0, yaw=0))
    automator.start(driver.drive_spline(spline))
    await forward(x=1)
    assert_pose(1, 0, deg=0)

    automator.pause(because='test')
    await forward(seconds=1.0)
    assert_pose(1, 0, deg=0)

    automator.resume()
    await forward(x=2)
    assert_pose(2, 0, deg=0)


@pytest.mark.parametrize('dx', [2, 10])
async def test_driving_a_spline(driver: Driver, automator: Automator, robot: Robot, dx: float):
    assert_pose(0, 0, deg=0)
    spline = Spline.from_poses(Pose(x=0, y=0, yaw=0), Pose(x=dx, y=1, yaw=0))
    automator.start(driver.drive_spline(spline))
    await forward(x=dx)
    assert_pose(dx, 1, deg=0, deg_tolerance=5)


async def test_aborting_a_drive(driver: Driver, automator: Automator, robot: Robot):
    assert_pose(0, 0, deg=0)
    automator.start(driver.drive_spline(Spline.from_poses(Pose(x=0), Pose(x=2))))
    cause: list[str] = []
    automator.AUTOMATION_STOPPED.subscribe(cause.append)
    await forward(x=1)
    assert_pose(1, 0, deg=0)
    driver.abort()
    await forward(seconds=1)
    assert_pose(1, 0, deg=0)
    assert cause == ['an exception occurred in an automation']


async def test_finally_block(automator: Automator):
    events: list[str] = []

    async def run() -> None:
        try:
            while True:
                events.append('tick')
                await rosys.sleep(3)
        finally:
            events.append('tock')

    automator.start(run())
    await forward(seconds=10)
    automator.stop(because='test')
    await forward(seconds=1)
    assert events == ['tick', 'tick', 'tick', 'tick', 'tock']


async def test_parallelize(automator: Automator):
    events: list[str] = []

    async def slow():
        try:
            for i in range(5):
                events.append(f'slow {i}')
                await rosys.sleep(0.5)
        finally:
            events.append('slow done')

    async def fast():
        try:
            for i in range(5):
                events.append(f'fast {i}')
                await rosys.sleep(0.2)
        finally:
            events.append('fast done')

    async def run(*, return_when_first_completed: bool):
        await rosys.automation.parallelize(slow(), fast(), return_when_first_completed=return_when_first_completed)

    events.clear()
    automator.start(run(return_when_first_completed=True))
    await forward(seconds=10)
    assert events == [
        'slow 0',
        'fast 0',
        'fast 1',
        'fast 2',
        'slow 1',
        'fast 3',
        'fast 4',
        'slow 2',
        'fast done',
        'slow done',
    ]
    assert automator.is_stopped

    events.clear()
    automator.start(run(return_when_first_completed=False))
    await forward(seconds=10)
    assert events == [
        'slow 0',
        'fast 0',
        'fast 1',
        'fast 2',
        'slow 1',
        'fast 3',
        'fast 4',
        'slow 2',
        'fast done',
        'slow 3',
        'slow 4',
        'slow done',
    ]
    assert automator.is_stopped


async def test_parallelize_exception(automator: Automator):
    failures: list[str] = []
    automator.AUTOMATION_FAILED.subscribe(failures.append)

    async def slow():
        for i in range(5):
            print(f'slow {i}')
            if i == 3:
                raise ValueError('i is 3')
            await rosys.sleep(0.5)

    async def fast():
        for i in range(5):
            print(f'fast {i}')
            await rosys.sleep(0.2)

    async def run():
        await rosys.automation.parallelize(slow(), fast())

    automator.start(run())
    await forward(seconds=10)
    assert failures == ['automation aborted because i is 3']


@pytest.mark.parametrize('method', ['pause', 'stop'])
async def test_uninterruptible(automator: Automator, method: Literal['pause', 'stop']):
    state = {'count': 0}

    async def a():
        for _ in range(10):
            await rosys.sleep(0.1)
            state['count'] += 1

    @rosys.automation.uninterruptible
    async def b():
        for _ in range(10):
            await rosys.sleep(0.1)
            state['count'] += 1

    async def run():
        await a()
        await b()
        await a()

    automator.start(run())
    await forward(seconds=1.5)
    if method == 'pause':
        automator.pause(because='we can')
    else:
        automator.stop(because='we can')
    await forward(seconds=2.0)
    assert state['count'] == 20
