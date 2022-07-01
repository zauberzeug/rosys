import pytest
from rosys import runtime
from rosys.test import forward


@pytest.mark.asyncio
async def test_time():
    assert runtime.time == 0.0

    await forward(until=1.0)
    assert runtime.time > 1.0

    await forward(seconds=1.0)
    assert runtime.time > 2.0

    await forward(3.0)
    assert runtime.time > 3.0
