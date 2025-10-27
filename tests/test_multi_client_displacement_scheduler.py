import asyncio

from attr import dataclass

import rosys
from rosys.helpers import MultiClientDisplacementScheduler
from rosys.testing import forward


@dataclass
class TestConfiguration:
    client_id: str
    delay: float
    task_id: str


async def run_displacement_test(running_slots: int, configurations: list[TestConfiguration],
                                expected: list[str | None]) -> None:
    async def shared_resource_access(identifier: str) -> str:
        await rosys.sleep(1)
        return identifier

    async def wait_and_append(configuration: TestConfiguration,
                              scheduler: MultiClientDisplacementScheduler) -> str | None:
        await rosys.sleep(configuration.delay)

        return await scheduler.run(configuration.client_id, shared_resource_access(configuration.task_id))

    async def advance_time(step: float, num_steps: int):
        for _ in range(num_steps):
            await forward(step)

    scheduler = MultiClientDisplacementScheduler(running_slots=running_slots)

    tasks = [wait_and_append(config, scheduler) for config in configurations]
    tasks.append(advance_time(0.1, 30))
    collected_tasks = asyncio.gather(*tasks)

    results = await collected_tasks
    results.pop()  # pop time task result
    assert results == expected


async def test_no_displacement() -> None:
    configs = [
        TestConfiguration('client0', 0.0, 'first'),
        TestConfiguration('client0', 0.2, 'final'),
    ]
    await run_displacement_test(1, configs, ['first', 'final'])


async def test_single_displacement() -> None:
    configs = [
        TestConfiguration('client0', 0.0, 'first'),
        TestConfiguration('client0', 0.1, 'displaced'),
        TestConfiguration('client0', 0.2, 'final'),
    ]
    await run_displacement_test(1, configs, ['first', None, 'final'])


async def test_multiple_displacement() -> None:
    configs = [
        TestConfiguration('client0', 0.0, 'first'),
        TestConfiguration('client0', 0.1, 'displaced'),
        TestConfiguration('client0', 0.2, 'displaced'),
        TestConfiguration('client0', 0.3, 'final'),
    ]
    await run_displacement_test(1, configs, ['first', None, None, 'final'])


async def test_multiple_displacement_two_slots() -> None:
    configs = [
        TestConfiguration('client0', 0.0, 'first'),
        TestConfiguration('client0', 0.1, 'second'),
        TestConfiguration('client0', 0.2, 'displaced'),
        TestConfiguration('client0', 0.3, 'final'),
    ]
    await run_displacement_test(2, configs, ['first', 'second', None, 'final'])


async def test_two_clients_one_slot() -> None:
    configs = [
        TestConfiguration('client0', 0.0, 'first0'),
        TestConfiguration('client0', 0.1, 'displaced'),
        TestConfiguration('client0', 0.2, 'final0'),
        TestConfiguration('client1', 0.1, 'displaced'),
        TestConfiguration('client1', 0.2, 'displaced'),
        TestConfiguration('client1', 0.3, 'final1'),
    ]
    await run_displacement_test(1, configs, ['first0', None, 'final0', None, None, 'final1'])


async def test_two_clients_two_slots() -> None:
    configs = [
        TestConfiguration('client0', 0.0, 'first0'),
        TestConfiguration('client0', 0.2, 'displaced'),
        TestConfiguration('client0', 0.3, 'final0'),
        TestConfiguration('client1', 0.1, 'first1'),
        TestConfiguration('client1', 0.2, 'displaced'),
        TestConfiguration('client1', 0.3, 'final1'),
    ]
    await run_displacement_test(2, configs, ['first0', None, 'final0', 'first1', None, 'final1'])
