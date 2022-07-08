from typing import Generator

import pytest
from rosys.actors import Automator, Driver, Odometer, PathPlanner
from rosys.hardware import Wheels, WheelsSimulation
from rosys.runtime import runtime
from rosys.test import helpers, log_configuration
from rosys.world import Robot

log_configuration.setup()


@pytest.fixture(autouse=True)  # HACK: use smallest scope to postpone runtime.startup until other fixtures are created
async def run_around_tests(odometer: Odometer, driver: Driver, automator: Automator) -> Generator:
    runtime.reset_for_test()
    helpers.odometer = odometer
    helpers.driver = driver
    helpers.automator = automator
    await runtime.startup()
    yield
    await runtime.shutdown()


@pytest.fixture(scope='class')
def odometer() -> Odometer:
    return Odometer()


@pytest.fixture(scope='class')
def wheels(odometer: Odometer) -> Wheels:
    return WheelsSimulation(odometer)


@pytest.fixture(scope='class')
def driver(wheels: Wheels) -> Driver:
    return Driver(wheels)


@pytest.fixture(scope='class')
def automator() -> Automator:
    return Automator()


@pytest.fixture(scope='class')
def robot() -> Robot:
    return Robot()


@pytest.fixture(scope='class')
def path_planner(robot: Robot) -> PathPlanner:
    return PathPlanner(robot.shape)
