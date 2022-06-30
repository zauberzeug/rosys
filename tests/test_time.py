import pytest
from rosys import core
from rosys.test import forward

from conftest import TestRuntime


@pytest.mark.asyncio
async def test_time(runtime: TestRuntime):
    assert core.time == 0.0

    await forward(until=1.0)
    assert core.time > 1.0

    await forward(seconds=1.0)
    assert core.time > 2.0

    await forward(3.0)
    assert core.time > 3.0
