import pytest

from rosys import run


@pytest.mark.asyncio
async def test_retry():
    events = []

    # retry three times with success
    async def func_successful():
        events.append('call')

    events.clear()
    success, result = await run.retry(func_successful)
    assert success is True
    assert result is None
    assert events == ['call']

    # retry three times with success and return value
    async def func_successful_with_result():
        events.append('call')
        return 'success'

    events.clear()
    success, result = await run.retry(func_successful_with_result)
    assert success is True
    assert result == 'success'
    assert events == ['call']

    # retry three times without success
    async def func_unsuccessful():
        events.append('call')
        raise ValueError()

    events.clear()
    success, result = await run.retry(func_unsuccessful)
    assert success is False
    assert result is None
    assert events == [
        'call',
        'call',
        'call',
    ]

    # with on_failed callback
    events.clear()
    success, result = await run.retry(func_unsuccessful, on_failed=lambda: events.append('failed'))
    assert success is False
    assert result is None
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
    success, result = await run.retry(func_unsuccessful, on_failed=lambda attempt, max_attempts: events.append(f'failed {attempt}/{max_attempts}'))
    assert success is False
    assert result is None
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
    success, result = await run.retry(func_unsuccessful, on_failed=handle_failed)
    assert success is False
    assert result is None
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
    success, result = await run.retry(func_unsuccessful, on_failed=handle_failed_with_args)
    assert success is False
    assert result is None
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
        success, result = await run.retry(func_unsuccessful, raise_on_failure=True)
    assert success is False
    assert result is None
    assert events == [
        'call',
        'call',
        'call',
    ]
