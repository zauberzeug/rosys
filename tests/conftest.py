from typing import Generator

import pytest
from rosys import event, lifecycle
from rosys.actors import Automator, Driver, Odometer
from rosys.core import set_time
from rosys.hardware import WheelsSimulation
from rosys.test import helper

import log_configuration

log_configuration.setup()


class TestRuntime:
    __test__ = False

    def __init__(self) -> None:
        self.odometer = Odometer()
        self.wheels = WheelsSimulation(self.odometer)
        self.driver = Driver(self.wheels)
        self.automator = Automator()

        helper.odometer = self.odometer
        helper.automator = self.automator
        helper.driver = self.driver


@pytest.fixture
async def runtime() -> Generator:
    event.listeners.clear()
    lifecycle.tasks.clear()
    runtime = TestRuntime()
    set_time(0)  # NOTE in tests we start at zero for better readability
    await lifecycle.startup()
    yield runtime
    await lifecycle.shutdown()
