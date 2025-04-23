import pytest

from rosys import run


@pytest.mark.asyncio
async def test_retry():
    events = []

    async def func():
        events.append('call')
        raise ValueError()

    # just retry three times
    events.clear()
    await run.retry(func)
    assert events == [
        'call',
        'call',
        'call',
    ]

    # with on_failed callback
    events.clear()
    await run.retry(func, on_failed=lambda: events.append('failed'))
    assert events == [
        'call',
        'failed',
        'call',
        'failed',
        'call',
        'failed',
    ]

    # with on_failed callback and attempt/max_attempts arguments
    events.clear()
    await run.retry(func, on_failed=lambda attempt, max_attempts: events.append(f'failed {attempt}/{max_attempts}'))
    assert events == [
        'call',
        'failed 0/3',
        'call',
        'failed 1/3',
        'call',
        'failed 2/3',
    ]

    # with async on_failed callback
    async def handle_failed():
        events.append('failed')

    events.clear()
    await run.retry(func, on_failed=handle_failed)
    assert events == [
        'call',
        'failed',
        'call',
        'failed',
        'call',
        'failed',
    ]

    # with async on_failed callback and attempt/max_attempts arguments
    async def handle_failed_with_args(*, attempt, max_attempts):
        events.append(f'failed {attempt}/{max_attempts}')

    events.clear()
    await run.retry(func, on_failed=handle_failed_with_args)
    assert events == [
        'call',
        'failed 0/3',
        'call',
        'failed 1/3',
        'call',
        'failed 2/3',
    ]

    # with raise_on_failure
    events.clear()
    with pytest.raises(RuntimeError):
        await run.retry(func, raise_on_failure=True)
    assert events == [
        'call',
        'call',
        'call',
    ]
