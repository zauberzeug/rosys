import time

import pytest

import rosys
from rosys.testing import forward


@pytest.mark.usefixtures('rosys_integration')
async def test_time():
    assert rosys.time() == 0.0

    await forward(until=1.0)
    assert rosys.time() == pytest.approx(1.0, abs=0.1)
    assert rosys.uptime() == pytest.approx(1.0, abs=0.1)

    await forward(seconds=1.0)
    assert rosys.time() == pytest.approx(2.0, abs=0.1)
    assert rosys.uptime() == pytest.approx(2.0, abs=0.1)

    await forward(3.0)
    assert rosys.time() == pytest.approx(5.0, abs=0.1)
    assert rosys.uptime() == pytest.approx(5.0, abs=0.1)


@pytest.mark.usefixtures('rosys_integration')
async def test_sleep():
    assert rosys.time() == 0.0
    sleep = rosys.background_tasks.create(rosys.sleep(1.0))
    await forward(0.5)
    assert not sleep.done()
    await forward(1.0)
    assert sleep.done()
    assert rosys.time() == pytest.approx(1.5)


@pytest.mark.usefixtures('rosys_integration')
async def test_sleep_wakes_on_time_while_cpu_bound_jobs_come_and_go():
    """A task sleeping in a loop while cpu_bound jobs of varying length start and end
    wakes up within one simulation step of its end time, every time."""
    duration, step = 0.05, 0.01
    overshoots: list[float] = []

    async def sleep_repeatedly() -> None:
        while True:
            start = rosys.time()
            await rosys.sleep(duration)
            overshoots.append(rosys.time() - start - duration)

    async def run_jobs() -> None:
        for i in range(40):
            await rosys.run.cpu_bound(time.sleep, 0.008 + 0.0005 * (i % 21))
            await rosys.sleep(0.03)

    rosys.background_tasks.create(sleep_repeatedly())
    jobs = rosys.background_tasks.create(run_jobs())
    await forward(until=jobs.done, dt=step)
    assert overshoots
    assert max(overshoots) <= step + 1e-9
