from typing import Generator
import pytest
import asyncio
import logging
import logging.config
import os
import sys
from rosys.runtime import Runtime, Mode
from rosys.world.world import World, WorldState
from rosys.world.robot import Robot
from rosys.world.marker import Marker

import icecream
icecream.install()


class PackagePathFilter(logging.Filter):
    # https://stackoverflow.com/a/52582536/3419103
    def filter(self, record):
        pathname = record.pathname
        record.relativepath = None
        abs_sys_paths = map(os.path.abspath, sys.path)
        for path in sorted(abs_sys_paths, key=len, reverse=True):  # longer paths first
            if not path.endswith(os.sep):
                path += os.sep
            if pathname.startswith(path):
                record.relativepath = os.path.relpath(pathname, path)
                break
        return True


config = {
    'version': 1,
    'disable_existing_loggers': True,
    'formatters': {
        'default': {
            'format': '%(asctime)s.%(msecs)03d [%(levelname)s] %(relativepath)s:%(lineno)d: %(message)s',
            'datefmt': '%Y-%m-%d %H:%M:%S',
        },
    },
    'filters': {
        'package_path_filter': {
            '()': PackagePathFilter,
        }
    },
    'handlers': {
        'console': {
            'class': 'logging.StreamHandler',
            'formatter': 'default',
            'filters': ['package_path_filter'],
            'level': 'INFO',
            'stream': 'ext://sys.stdout'
        },
    },
    'loggers': {
        '': {  # root logger
            'handlers': ['console'],
            'level': 'WARN'
        },
        'rosys': {
            'handlers': ['console'],
            'level': 'DEBUG'
        },
        'rosys.ota': {
            'handlers': ['console'],
            'level': 'WARN'
        }
    },
}

logging.config.dictConfig(config)


pytest.register_assert_rewrite("tests.helper")


@pytest.fixture
async def runtime() -> Generator:
    runtime = Runtime(World(
        mode=Mode.TEST,
        state=WorldState.RUNNING,
        robot=Robot(marker=Marker(points={'front': (0.12, 0), 'back': (-0.12, 0)}, height=0.58)),
    )).with_cameras()

    from tests.helper import set_global_runtime
    set_global_runtime(runtime)

    runtime.world.set_time(0)  # NOTE in tests we start at zero for better reading

    async def forward(seconds, dt=0.01):
        # NOTE we start runtime here because this makes it easy in the tests to prepare it beforehand
        if not runtime.tasks:
            await runtime.run()

        end_time = runtime.world.time + seconds
        while runtime.world.time <= end_time:
            runtime.world.set_time(runtime.world.time + dt)
            await asyncio.sleep(0)

    runtime.forward = forward

    async def sleep(seconds: float):
        sleep_end_time = runtime.world.time + seconds
        while runtime.world.time <= sleep_end_time:
            await asyncio.sleep(0)

    for a in runtime.actors:
        a.sleep = sleep

    yield runtime
    await runtime.stop()
