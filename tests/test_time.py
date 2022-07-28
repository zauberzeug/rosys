import pytest
import rosys
from rosys.test import forward


@pytest.mark.asyncio
async def test_time():
    assert rosys.time() == 0.0

    await forward(until=1.0)
    assert rosys.time() > 1.0

    await forward(seconds=1.0)
    assert rosys.time() > 2.0

    await forward(3.0)
    assert rosys.time() > 3.0
