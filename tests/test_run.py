import pytest

from rosys import run


@pytest.mark.asyncio
async def test_retry():
    events = []

    async def func():
        events.append('call')
        raise ValueError()

    # with logging callback
    events.clear()
    await run.retry(func, logging_callback=lambda attempt, max_attempts: events.append(f'failed {attempt}/{max_attempts}'))
    assert events == [
        'call',
        'failed 0/3',
        'call',
        'failed 1/3',
        'call',
        'failed 2/3',
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

    # with raise_on_failure
    events.clear()
    with pytest.raises(RuntimeError):
        await run.retry(func, raise_on_failure=True)
    assert events == [
        'call',
        'call',
        'call',
    ]
