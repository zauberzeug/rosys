import pytest

from rosys import run


@pytest.mark.asyncio
async def test_retry():
    events = []

    # retry three times with success
    async def func_successful():
        events.append('call')

    events.clear()
    await run.retry(func_successful)
    assert events == ['call']

    # retry three times with success and return value
    async def func_successful_with_result() -> str:
        events.append('call')
        return 'success'

    events.clear()
    result = await run.retry(func_successful_with_result)
    assert result == 'success'
    assert events == ['call']

    # retry three times without success
    async def func_unsuccessful():
        events.append('call')
        raise ValueError()

    events.clear()
    try:
        await run.retry(func_unsuccessful)
    except RuntimeError:
        pass
    else:
        raise AssertionError('Expected RuntimeError')
    assert events == [
        'call',
        'call',
        'call',
    ]

    # with on_failed callback
    events.clear()
    try:
        await run.retry(func_unsuccessful, on_failed=lambda: events.append('failed'))
    except RuntimeError:
        pass
    else:
        raise AssertionError('Expected RuntimeError')
    assert events == [
        'call',
        'failed',
        'call',
        'failed',
        'call',
        'failed',
    ]

    # with on_failed callback and used arguments
    events.clear()
    try:
        await run.retry(func_unsuccessful, on_failed=lambda args: events.append(f'failed {args.attempt}/{args.max_attempts}'))
    except RuntimeError:
        pass
    else:
        raise AssertionError('Expected RuntimeError')
    assert events == [
        'call',
        'failed 0/3',
        'call',
        'failed 1/3',
        'call',
        'failed 2/3',
    ]
