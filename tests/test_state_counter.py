import asyncio

import pytest

from rosys.hardware import StateCounter
from rosys.testing import forward


def test_a_command_taken_and_ended_between_two_reports_counts():
    counter = StateCounter(3)
    base = counter.command()
    counter.value = 3
    assert counter.taken(base)
    assert counter.ended(base)


def test_a_refused_command_never_shows():
    counter = StateCounter(3)
    counter.value = 6
    base = counter.command()
    assert not counter.taken(base)


def test_the_end_of_a_command_in_flight_is_not_the_next_ones():
    counter = StateCounter(3)
    counter.value = 2
    base = counter.command()
    counter.value = 3
    assert not counter.taken(base)
    counter.value = 5
    assert counter.taken(base)
    assert not counter.ended(base)
    counter.value = 6
    assert counter.ended(base)


def test_commands_sent_before_the_last_one_showed_queue_behind_it():
    counter = StateCounter(3)
    first = counter.command()
    second = counter.command()
    counter.value = 3
    assert counter.ended(first)
    assert not counter.taken(second)
    counter.value = 6
    assert counter.ended(second)


def test_a_reset_counts_from_the_reported_counter_again():
    counter = StateCounter(3)
    counter.command()
    counter.reset()
    base = counter.command()
    counter.value = 3
    assert counter.ended(base)


def test_a_counter_far_behind_the_commands_means_lizard_restarted():
    counter = StateCounter(3)
    counter.value = 120
    counter.command()
    counter.value = 0
    base = counter.command()
    counter.value = 3
    assert counter.ended(base)


def test_the_counter_wraps_at_a_multiple_of_the_states():
    counter = StateCounter(4)
    assert counter.period == 252
    counter.value = 251
    assert counter.state == 3
    base = counter.command()
    assert base == 0
    counter.value = 4
    assert counter.ended(base)


@pytest.mark.usefixtures('rosys_integration')
async def test_waiting_for_a_command_returns_once_the_counter_shows_it():
    counter = StateCounter(3)
    base = counter.command()
    waiting = asyncio.ensure_future(counter.wait_ended(base, timeout=1.0))
    await forward(0.2)
    assert not waiting.done()
    counter.value = 3
    await forward(0.1)
    assert waiting.done() and waiting.result()


@pytest.mark.usefixtures('rosys_integration')
async def test_waiting_for_a_command_that_never_shows_times_out():
    counter = StateCounter(3)
    waiting = asyncio.ensure_future(counter.wait_taken(counter.command(), timeout=0.5))
    await forward(1.0)
    assert waiting.done() and not waiting.result()
