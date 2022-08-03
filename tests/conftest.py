from typing import Generator

import pytest
import rosys
from rosys.automation import Automator
from rosys.driving import Driver, Odometer
from rosys.geometry import Prism
from rosys.hardware import Wheels, WheelsSimulation
from rosys.pathplanning import PathPlanner
from rosys.test import helpers, log_configuration

log_configuration.setup()


@pytest.fixture(autouse=True)
async def run_around_tests(odometer: Odometer, driver: Driver, automator: Automator) -> Generator:
    rosys.reset_before_test()
    helpers.odometer = odometer
    helpers.driver = driver
    helpers.automator = automator
    await rosys.startup()
    yield
    await rosys.shutdown()
    rosys.reset_after_test()


@pytest.fixture(autouse=True)
def odometer(wheels: Wheels) -> Odometer:
    return Odometer(wheels)


@pytest.fixture(autouse=True)
def wheels() -> Wheels:
    return WheelsSimulation()


@pytest.fixture(autouse=True)
def driver(wheels: Wheels, odometer: Odometer) -> Driver:
    return Driver(wheels, odometer)


@pytest.fixture(autouse=True)
def automator(wheels: Wheels) -> Automator:
    return Automator(wheels, None)


@pytest.fixture(autouse=True)
def shape() -> Prism:
    return Prism.default_robot_shape()


@pytest.fixture(autouse=True)
def path_planner(shape: Prism) -> PathPlanner:
    return PathPlanner(shape)
