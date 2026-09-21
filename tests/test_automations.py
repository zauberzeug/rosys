import asyncio
import time
from typing import Literal

import numpy as np
import pytest

import rosys
from rosys.automation import Automator
from rosys.automation.automation import Automation
from rosys.driving import Driver
from rosys.geometry import Pose, Spline
from rosys.hardware import Robot, Wheels
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


@pytest.mark.parametrize('dx', [2, -2, 10])
async def test_driving_a_spline(driver: Driver, automator: Automator, robot: Robot, dx: float):
    assert_pose(0, 0, deg=0)
    spline = Spline.from_poses(Pose(x=0, y=0, yaw=0), Pose(x=dx, y=1, yaw=0), backward=dx < 0)
    automator.start(driver.drive_spline(spline, flip_hook=dx < 0))
    await forward(until=lambda: automator.is_running)
    await forward(until=lambda: automator.is_stopped)
    assert_pose(dx, 1, deg=0, position_tolerance=0.035)


@pytest.mark.parametrize('dx', [2, -2])
async def test_driving_a_curved_spline(driver: Driver, automator: Automator, robot: Robot, dx: float):
    assert_pose(0, 0, deg=0)
    yaw_degrees = 90 if dx > 0 else -90
    spline = Spline.from_poses(Pose(x=0, y=0, yaw=0), Pose(x=dx, y=2, yaw=np.radians(yaw_degrees)), backward=dx < 0)
    automator.start(driver.drive_spline(spline, flip_hook=dx < 0))
    await forward(until=lambda: automator.is_running)
    await forward(until=lambda: automator.is_stopped)
    assert_pose(dx, 2, deg=yaw_degrees, position_tolerance=0.035)


async def test_aborting_a_drive(driver: Driver, automator: Automator, robot: Robot):
    assert_pose(0, 0, deg=0)
    automator.start(driver.drive_spline(Spline.from_poses(Pose(x=0), Pose(x=2))))
    cause: list[str] = []
    automator.AUTOMATION_FAILED.subscribe(cause.append)
    await forward(x=1)
    assert_pose(1, 0, deg=0)
    driver.abort()
    await forward(seconds=1, fail_on_automation_failure=False)
    assert_pose(1, 0, deg=0)
    assert cause == ['an exception occurred in an automation']


async def test_a_raising_automation_stops_forwarding(automator: Automator):
    """Forwarding stops at the failure instead of stepping through the whole span it was given."""
    async def run() -> None:
        raise RuntimeError('the automation broke')
    automator.start(run())
    started_at = rosys.time()
    with pytest.raises(AssertionError, match='the automation broke') as exception_info:
        await forward(seconds=60)
    assert rosys.time() - started_at == pytest.approx(0, abs=0.1)
    assert isinstance(exception_info.value.__cause__, RuntimeError), 'the original failure stays attached'


async def test_a_new_automation_forwards_past_an_earlier_failure(automator: Automator):
    """A failure belongs to the automation that raised it, so the next run forwards freely."""
    async def failing() -> None:
        raise RuntimeError('the automation broke')

    completed = False

    async def working() -> None:
        nonlocal completed
        await rosys.sleep(1.0)
        completed = True

    automator.start(failing())
    await forward(seconds=1, fail_on_automation_failure=False)
    automator.start(working())
    await forward(seconds=2)
    assert completed


async def test_a_failure_during_the_wait_stops_forwarding(automator: Automator):
    """Forwarding stops even when the failure satisfies the condition it is waiting for."""
    async def failing() -> None:
        await rosys.sleep(0.5)
        raise RuntimeError('the automation broke')
    automator.start(failing())
    # the automation counts as stopped until its task has started, which would satisfy the condition right away
    await forward(seconds=0.1)
    with pytest.raises(AssertionError, match='the automation broke'):
        await forward(until=lambda: automator.is_stopped)


async def test_a_failure_stops_forwarding_to_a_condition_that_already_holds(automator: Automator):
    """Forwarding stops before waiting, not only in between two steps."""
    async def failing() -> None:
        raise RuntimeError('the automation broke')
    automator.start(failing())
    await forward(seconds=1, fail_on_automation_failure=False)
    with pytest.raises(AssertionError, match='the automation broke'):
        await forward(until=lambda: True)


async def _count(ticks: list[int]) -> None:
    while True:
        await rosys.sleep(1)
        ticks.append(1)


@pytest.mark.parametrize('gap', [2.0, 0.0], ids=['over a running automation', 'in the same loop turn'])
async def test_stopping_an_automation_that_was_started_over_another_one(automator: Automator, gap: float):
    """``start()`` over a running (#461) or a not yet started automation must not orphan the new one."""
    ticks: list[int] = []
    events: list[str] = []
    automator.AUTOMATION_STARTED.subscribe(lambda: events.append('started'))
    automator.AUTOMATION_STOPPED.subscribe(lambda _: events.append('stopped'))

    automator.start(_count(ticks))
    if gap > 0:
        await forward(seconds=gap)
    automator.start(_count(ticks))
    await forward(seconds=2)
    assert automator.is_running
    automator.stop(because='test')
    ticks_at_stop = len(ticks)
    await forward(seconds=5)
    assert len(ticks) == ticks_at_stop, 'an automation kept running after stop()'
    assert automator.is_stopped
    assert events == ['started', 'stopped', 'started', 'stopped']


@pytest.mark.parametrize('method', ['stop', 'abort'])
async def test_stopping_an_automation_that_has_not_started_yet(automator: Automator, method: Literal['stop', 'abort']):
    ticks: list[int] = []
    automator.start(_count(ticks))
    assert automator.is_pending
    getattr(automator, method)(because='test')
    assert not automator.is_pending
    await forward(seconds=3)
    assert not ticks
    assert automator.is_stopped


@pytest.mark.parametrize('old_state', ['running', 'stopping'])
async def test_a_superseded_automation_does_not_override_the_new_drive_command(automator: Automator, wheels: Wheels,
                                                                               old_state: Literal['running', 'stopping']):
    """``on_interrupt`` of the old automation must land before the new one's first drive command, or not at all."""
    async def old() -> None:
        try:
            await wheels.drive(0.3, 0)
            await rosys.sleep(100)
        finally:
            await rosys.sleep(1)

    async def new() -> None:
        await wheels.drive(0.5, 0)
        await rosys.sleep(100)

    automator.start(old())
    await forward(seconds=1)
    if old_state == 'stopping':
        automator.stop(because='test')
        await forward(seconds=0.5)
        assert automator.is_stopping
    assert wheels.linear_target_speed == 0.3
    automator.start(new())
    await forward(seconds=2)
    assert wheels.linear_target_speed == 0.5, 'the old automation stopped the wheels of the new one'


async def test_a_superseded_automation_stops_the_wheels_the_new_one_leaves_alone(automator: Automator, wheels: Wheels):
    """``on_interrupt`` of the old automation runs before the new automation's first turn."""
    async def old() -> None:
        try:
            await wheels.drive(0.3, 0)
            await rosys.sleep(100)
        finally:
            await rosys.sleep(1)

    async def new() -> None:
        await rosys.sleep(100)

    automator.start(old())
    await forward(seconds=1)
    assert wheels.linear_target_speed == 0.3
    automator.start(new())
    await forward(seconds=2)
    assert wheels.linear_target_speed == 0.0, 'the superseded automation never stopped the wheels'


async def test_an_exception_in_the_cleanup_of_a_superseded_automation_is_ignored(automator: Automator):
    """An exception while the old automation cleans up must neither abort the new one nor be attributed to it."""
    ticks: list[int] = []

    async def old() -> None:
        try:
            await rosys.sleep(100)
        finally:
            await rosys.sleep(3)
            raise RuntimeError('cleanup of the old automation failed')

    automator.start(old())
    await forward(seconds=1)
    automator.start(_count(ticks))
    await forward(seconds=4)
    ticks_after_exception = len(ticks)
    await forward(seconds=2)
    assert len(ticks) > ticks_after_exception, 'the new automation was aborted'
    assert automator.is_running
    assert automator.last_exception is None


async def test_starting_over_a_pausing_automation(automator: Automator):
    """A pending pause inside an uninterruptible section must not leave the old automation parked forever."""
    events: list[str] = []

    @rosys.automation.uninterruptible
    async def uninterruptible_section() -> None:
        await rosys.sleep(1)

    async def old() -> None:
        try:
            await uninterruptible_section()
            events.append('old continued')
            await rosys.sleep(100)
        finally:
            events.append('old finished')

    async def new() -> None:
        await rosys.sleep(100)

    automator.start(old())
    await forward(seconds=0.5)
    automator.pause(because='test')
    assert automator.is_pausing
    automator.start(new())
    await forward(seconds=2)
    assert events == ['old finished']
    assert automator.is_running


async def test_an_automation_that_starts_another_one_and_returns(automator: Automator):
    """The old automation completes after ``start()`` replaced it, which must not clear the handle or emit COMPLETED."""
    ticks: list[int] = []
    completed: list[int] = []
    automator.AUTOMATION_COMPLETED.subscribe(lambda: completed.append(1))

    async def old() -> None:
        automator.start(_count(ticks))

    automator.start(old())
    await forward(seconds=3)
    assert automator.is_running
    assert not completed
    automator.stop(because='test')
    ticks_at_stop = len(ticks)
    await forward(seconds=3)
    assert len(ticks) == ticks_at_stop, 'the automation kept running after stop()'


async def test_starting_paused_while_a_subscriber_starts_another_automation(automator: Automator):
    """``paused=True`` must pause the automation that was started, not one started re-entrantly by a subscriber."""
    ticks: list[int] = []
    started = 0

    def start_another() -> None:
        nonlocal started
        started += 1
        if started == 1:
            automator.start(_count(ticks))

    automator.AUTOMATION_STARTED.subscribe(start_another)

    async def main() -> None:
        await rosys.sleep(100)

    automator.start(main(), paused=True)
    await forward(seconds=3)
    assert automator.is_running, 'the re-entrantly started automation was paused instead'
    assert ticks


async def test_stopping_a_pausing_automation(automator: Automator):
    """A stop during a pending pause inside an uninterruptible section is executed, not ignored."""
    events: list[str] = []

    @rosys.automation.uninterruptible
    async def uninterruptible_section() -> None:
        await rosys.sleep(1)

    async def run() -> None:
        try:
            await uninterruptible_section()
            events.append('continued')
            await rosys.sleep(100)
        finally:
            events.append('finished')

    automator.start(run())
    await forward(seconds=0.5)
    automator.pause(because='test')
    assert automator.is_pausing
    automator.stop(because='test')
    await forward(seconds=3)
    assert events == ['finished']
    assert automator.is_stopped


async def test_finally_block(automator: Automator):
    events: list[str] = []

    async def run() -> None:
        try:
            while True:
                events.append('tick')
                await rosys.sleep(3)
        finally:
            events.append('tock')

    pinned: list = []  # prevent refcount-driven GC from masking the bug
    for _ in range(2):
        coro = run()
        pinned.append(coro)
        automator.start(coro)
        await forward(seconds=10)
        pinned.append(automator.automation)
        automator.stop(because='test')
        await forward(seconds=1)
    assert events == ['tick', 'tick', 'tick', 'tick', 'tock'] * 2


async def test_async_finally_block_on_stop(automator: Automator):
    """Stopping an automation must let async cleanup in ``finally`` blocks run to completion.

    Regression test for the PR #403 tradeoff: ``coro.close()`` injected a ``GeneratorExit``,
    which forbids ``await`` during cleanup ("coroutine ignored GeneratorExit").
    """
    events: list[str] = []

    async def run() -> None:
        try:
            while True:
                events.append('tick')
                await rosys.sleep(3)
        finally:
            events.append('cleanup start')
            await rosys.sleep(1)  # async cleanup must be allowed to run
            events.append('cleanup done')

    pinned: list = []  # prevent refcount-driven GC from masking the bug
    coro = run()
    pinned.append(coro)
    automator.start(coro)
    await forward(seconds=5)
    pinned.append(automator.automation)
    automator.stop(because='test')
    await forward(seconds=3)
    assert events == ['tick', 'tick', 'cleanup start', 'cleanup done']
    assert automator.is_stopped


async def test_async_finally_block_in_parallelize_on_stop(automator: Automator):
    """Async cleanup must also run when stopping a parallelized coroutine."""
    events: list[str] = []

    async def worker() -> None:
        try:
            while True:
                events.append('tick')
                await rosys.sleep(3)
        finally:
            events.append('cleanup start')
            await rosys.sleep(1)
            events.append('cleanup done')

    async def run() -> None:
        await rosys.automation.parallelize(worker(), return_when_first_completed=True)

    automator.start(run())
    await forward(seconds=5)
    automator.stop(because='test')
    await forward(seconds=3)
    assert events == ['tick', 'tick', 'cleanup start', 'cleanup done']
    assert automator.is_stopped


async def test_async_finally_runs_when_stopped_while_parked_on_future(automator: Automator):
    """Production-faithful variant: the coroutine parks on a real ``Future`` (``parallelize``'s bare yield),
    not on the ``asyncio.sleep(0)`` of test-mode ``rosys.sleep``. The stop must still drive async cleanup.
    """
    events: list[str] = []

    class ParkOnFuture:
        """Awaitable that yields an unresolved ``Future`` like production ``asyncio.sleep``, resolved next tick."""

        def __await__(self):
            loop = asyncio.get_event_loop()
            future = loop.create_future()
            loop.call_soon(future.set_result, None)
            return (yield from future.__await__())

    async def worker() -> None:
        try:
            while True:
                events.append('tick')
                await ParkOnFuture()
        finally:
            events.append('cleanup start')
            await ParkOnFuture()
            events.append('cleanup done')

    async def run() -> None:
        await rosys.automation.parallelize(worker(), return_when_first_completed=True)

    automator.start(run())
    await forward(until=lambda: events.count('tick') >= 2)
    automator.stop(because='test')
    await forward(until=lambda: automator.is_stopped, timeout=5)
    assert events == ['tick', 'tick', 'cleanup start', 'cleanup done']


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
    await forward(seconds=10, fail_on_automation_failure=False)
    assert failures == ['an exception occurred in an automation: i is 3']


async def test_parallelize_suspends_while_all_coroutines_are_parked_on_futures():
    """``parallelize`` must wait on the coroutines' futures instead of busy-polling the event loop (regression).

    This runs on real time because test-mode ``rosys.sleep`` never parks on a future.
    """
    events: list[str] = []

    async def fast() -> None:
        await asyncio.sleep(0.1)
        events.append('fast done')

    async def slow() -> None:
        try:
            await asyncio.sleep(5.0)
            events.append('slow done')
        finally:
            events.append('slow cleanup')

    cpu_start = time.process_time()
    wall_start = time.monotonic()
    await rosys.automation.parallelize(slow(), fast(), return_when_first_completed=True)
    assert time.monotonic() - wall_start < 1.0, 'should return as soon as the fast coroutine completes'
    assert time.process_time() - cpu_start < 0.05, 'should suspend instead of burning CPU while waiting'
    assert events == ['fast done', 'slow cleanup']


async def test_automation_can_wrap_a_non_coroutine_awaitable():
    """``Automation`` must also close awaitables like ``parallelize`` which are not coroutines (regression)."""
    events: list[str] = []

    async def worker() -> None:
        await asyncio.sleep(0.01)
        events.append('done')

    assert await Automation(rosys.automation.parallelize(worker())).run() is None
    assert events == ['done']


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
