import pytest

import rosys
from rosys.test import forward


@pytest.mark.usefixtures('integration')
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


@pytest.mark.usefixtures('integration')
async def test_sleep():
    assert rosys.time() == 0.0
    sleep = rosys.background_tasks.create(rosys.sleep(1.0))
    await forward(0.5)
    assert not sleep.done()
    await forward(1.0)
    assert sleep.done()
    assert rosys.time() == pytest.approx(1.5)
