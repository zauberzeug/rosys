# pylint: disable=redefined-outer-name,unused-argument
import asyncio
import multiprocessing
from collections.abc import AsyncGenerator

import pytest
from nicegui import core

import rosys
from rosys.analysis import KpiLogger
from rosys.automation import Automator
from rosys.driving import Driver, Odometer
from rosys.geometry import GeoPoint, GeoReference, Prism
from rosys.hardware import Robot, RobotSimulation, Wheels, WheelsSimulation
from rosys.pathplanning import PathPlanner
from rosys.testing import helpers, log_configuration


@pytest.fixture
async def rosys_integration() -> AsyncGenerator:
    log_configuration.setup()
    core.loop = asyncio.get_event_loop()
    rosys.reset_before_test()
    await rosys.startup()
    yield
    await rosys.shutdown()
    rosys.reset_after_test()


@pytest.fixture
async def odometer(wheels: Wheels, rosys_integration: None) -> Odometer:
    helpers.odometer = Odometer(wheels)
    return helpers.odometer


@pytest.fixture
async def wheels(rosys_integration: None) -> Wheels:
    return WheelsSimulation()


@pytest.fixture
async def robot(wheels: Wheels, rosys_integration: None) -> Robot:
    return RobotSimulation([wheels])


@pytest.fixture
async def driver(wheels: Wheels, odometer: Odometer, rosys_integration: None) -> Driver:
    helpers.driver = Driver(wheels, odometer)
    return helpers.driver


@pytest.fixture
async def automator(wheels: Wheels, rosys_integration: None) -> Automator:
    helpers.automator = Automator(None, on_interrupt=wheels.stop)
    return helpers.automator


@pytest.fixture
def shape() -> Prism:
    return Prism.default_robot_shape()


@pytest.fixture
async def path_planner(shape: Prism, rosys_integration: None) -> PathPlanner:
    return PathPlanner(shape)


@pytest.fixture
async def kpi_logger(rosys_integration: None) -> KpiLogger:
    return KpiLogger()


@pytest.fixture(scope='session', autouse=True)
def enforce_spawn_process() -> None:
    if multiprocessing.get_start_method() != 'spawn':
        multiprocessing.set_start_method('spawn', force=True)


@pytest.fixture
def geo_reference() -> None:
    geo_reference = GeoReference(GeoPoint.from_degrees(lat=0, lon=0))
    GeoReference.update_current(geo_reference)
