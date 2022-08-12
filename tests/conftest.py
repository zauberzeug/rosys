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


@pytest.fixture(autouse=False)
async def integration() -> Generator:
    rosys.reset_before_test()
    await rosys.startup()
    yield
    await rosys.shutdown()
    rosys.reset_after_test()


@pytest.fixture(autouse=False)
async def odometer(wheels: Wheels, integration: None) -> Odometer:
    helpers.odometer = Odometer(wheels)
    return helpers.odometer


@pytest.fixture(autouse=False)
async def wheels(integration: None) -> Wheels:
    return WheelsSimulation()


@pytest.fixture(autouse=False)
async def driver(wheels: Wheels, odometer: Odometer, integration: None) -> Driver:
    helpers.driver = Driver(wheels, odometer)
    return helpers.driver


@pytest.fixture(autouse=False)
async def automator(wheels: Wheels, integration: None) -> Automator:
    helpers.automator = Automator(wheels, None)
    return helpers.automator


@pytest.fixture(autouse=False)
def shape() -> Prism:
    return Prism.default_robot_shape()


@pytest.fixture(autouse=False)
async def path_planner(shape: Prism, integration: None) -> PathPlanner:
    return PathPlanner(shape)
