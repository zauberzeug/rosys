import pytest

from rosys import run, sleep
from rosys.automation import Automator
from rosys.testing import forward


@pytest.mark.asyncio
async def test_timeouts(automator: Automator):
    async def func() -> str:
        await sleep(0.5)
        return 'success'

    failures = []
    automator.AUTOMATION_FAILED.subscribe(failures.append)

    automator.start(run.wait_for(func, timeout=1.0))
    await forward(seconds=2.0)
    assert automator.is_stopped
    assert len(failures) == 0

    automator.start(run.wait_for(func, timeout=0.25))
    await forward(seconds=2.0)
    assert automator.is_stopped
    assert len(failures) == 1

    automator.start(run.retry(func, max_attempts=1, max_timeout=1.0))
    await forward(seconds=2.0)
    assert automator.is_stopped
    assert len(failures) == 1

    automator.start(run.retry(func, max_attempts=1, max_timeout=0.25))
    await forward(seconds=2.0)
    assert automator.is_stopped
    assert len(failures) == 2


@pytest.mark.asyncio
async def test_retry():
    async def func() -> str:
        events.append('call')
        return 'success'

    events: list[str] = []
    result = await run.retry(func)
    assert result == 'success'
    assert events == ['call']


@pytest.mark.asyncio
async def test_retry_failed():
    async def func() -> None:
        events.append('call')
        raise ValueError()

    events: list[str] = []
    with pytest.raises(RuntimeError):
        await run.retry(func)
    assert events == ['call'] * 3


@pytest.mark.asyncio
async def test_retry_failed_with_on_failed():
    async def func() -> None:
        events.append('call')
        raise ValueError()

    events: list[str] = []
    with pytest.raises(RuntimeError):
        await run.retry(func, on_failed=lambda: events.append('failed'))
    assert events == ['call', 'failed'] * 3


@pytest.mark.asyncio
async def test_retry_failed_with_on_failed_and_args():
    async def func() -> None:
        events.append('call')
        raise ValueError()

    events: list[str] = []
    with pytest.raises(RuntimeError):
        await run.retry(func, on_failed=lambda args: events.append(f'failed {args.attempt}/{args.max_attempts}'))
    assert events == ['call', 'failed 0/3',
                      'call', 'failed 1/3',
                      'call', 'failed 2/3']
