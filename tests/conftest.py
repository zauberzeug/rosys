from typing import Generator

import pytest
from rosys.actors import Automator, Driver, Odometer, PathPlanner
from rosys.hardware import Wheels, WheelsSimulation
from rosys.runtime import runtime
from rosys.test import helpers, log_configuration
from rosys.world import RobotShape

log_configuration.setup()


@pytest.fixture(autouse=True)
async def run_around_tests(odometer: Odometer, driver: Driver, automator: Automator) -> Generator:
    runtime.reset_before_test()
    helpers.odometer = odometer
    helpers.driver = driver
    helpers.automator = automator
    await runtime.startup()
    yield
    await runtime.shutdown()
    runtime.reset_after_test()


@pytest.fixture(autouse=True)
def odometer() -> Odometer:
    return Odometer()


@pytest.fixture(autouse=True)
def wheels(odometer: Odometer) -> Wheels:
    return WheelsSimulation(odometer)


@pytest.fixture(autouse=True)
def driver(wheels: Wheels) -> Driver:
    return Driver(wheels)


@pytest.fixture(autouse=True)
def automator() -> Automator:
    return Automator()


@pytest.fixture(autouse=True)
def shape() -> RobotShape:
    return RobotShape()


@pytest.fixture(autouse=True)
def path_planner(shape: RobotShape) -> PathPlanner:
    return PathPlanner(shape)
