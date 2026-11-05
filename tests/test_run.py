import asyncio
import threading
from unittest.mock import patch

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
    await forward(seconds=2.0, fail_on_automation_failure=False)
    assert automator.is_stopped
    assert len(failures) == 1

    automator.start(run.retry(func, max_attempts=1, max_timeout=1.0))
    await forward(seconds=2.0)
    assert automator.is_stopped
    assert len(failures) == 1

    automator.start(run.retry(func, max_attempts=1, max_timeout=0.25))
    await forward(seconds=2.0, fail_on_automation_failure=False)
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


@pytest.mark.parametrize('bound', ['io_bound', 'cpu_bound'])
async def test_a_call_refused_by_the_shut_down_pools_returns_none(bound: str):
    async def refuse(*_) -> None:
        raise asyncio.CancelledError('The app has shut down.')

    with patch(f'nicegui.run.{bound}', refuse):
        assert await getattr(run, bound)(print) is None
    assert not run.running_cpu_bound_processes


@pytest.mark.parametrize('bound', ['io_bound', 'cpu_bound'])
async def test_cancelling_the_task_propagates(bound: str):
    async def wait_forever(*_) -> None:
        await asyncio.Event().wait()

    with patch(f'nicegui.run.{bound}', wait_forever):
        task = asyncio.create_task(getattr(run, bound)(print))
        await asyncio.sleep(0)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
    assert task.cancelled()
    assert not run.running_cpu_bound_processes


async def test_cancelling_a_running_io_bound_call_does_not_wait_for_the_thread():
    release = threading.Event()
    task = asyncio.create_task(run.io_bound(release.wait))
    await asyncio.sleep(0.01)
    task.cancel()
    try:
        with pytest.raises(asyncio.CancelledError):
            await task
    finally:
        release.set()
