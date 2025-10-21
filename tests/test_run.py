import asyncio

import pytest

from rosys import run


@pytest.mark.asyncio
async def test_wait_for_success():
    async def func() -> str:
        await asyncio.sleep(0.5)
        return 'success'
    result = await run.wait_for(func, timeout=1.0)
    assert result == 'success'


@pytest.mark.asyncio
async def test_wait_for_timeout():
    async def func() -> None:
        await asyncio.sleep(1.0)

    with pytest.raises(TimeoutError):
        await run.wait_for(func, timeout=0.5)


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
async def test_retry_timeout():
    async def func() -> str:
        await asyncio.sleep(0.5)
        return 'success'

    result = await run.retry(func, max_attempts=1, max_timeout=1.0)
    assert result == 'success'


@pytest.mark.asyncio
async def test_retry_timeout_failed():
    async def func() -> None:
        await asyncio.sleep(1.0)

    with pytest.raises(RuntimeError):
        await run.retry(func, max_attempts=1, max_timeout=0.5)


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
