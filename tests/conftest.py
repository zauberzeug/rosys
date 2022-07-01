from typing import Generator

import pytest
from rosys import event, runtime
from rosys.actors import Automator, Driver, Odometer
from rosys.hardware import WheelsSimulation
from rosys.test import helpers

import log_configuration

log_configuration.setup()


@pytest.fixture(autouse=True)
async def run_around_tests():
    runtime.set_time(0)  # NOTE in tests we start at zero for better readability
    event.listeners.clear()
    helpers.odometer = Odometer()
    helpers.wheels = WheelsSimulation(helpers.odometer)
    helpers.driver = Driver(helpers.wheels)
    helpers.automator = Automator()
    await runtime.startup()
    yield
    await runtime.shutdown()


@pytest.fixture
async def driver() -> Generator:
    return helpers.driver


@pytest.fixture
async def automator() -> Generator:
    return helpers.automator


@pytest.fixture
async def wheels() -> Generator:
    return helpers.wheels
