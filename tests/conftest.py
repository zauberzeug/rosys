from typing import Generator
import pytest
import logging
import logging.config
import os
import sys
from rosys.test.runtime import TestRuntime

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


class RuntimeFilter(logging.Filter):
    def filter(self, record):
        from rosys.test.helper import global_runtime
        if global_runtime:
            record.worldtime = global_runtime.world.time
            pose = global_runtime.world.robot.prediction
            record.robotpose = '% 1.2f, % 1.2f, % 1.2f' % (pose.x, pose.y, pose.yaw_deg)
        else:
            record.worldtime = 0
            record.robotpose = 'no robot pose yet'
        return True


config = {
    'version': 1,
    'disable_existing_loggers': True,
    'formatters': {
        'default': {
            'format': '%(worldtime).2f [%(levelname)s] %(robotpose)s %(relativepath)s:%(lineno)d: %(message)s',
            'datefmt': '%Y-%m-%d %H:%M:%S',
        },
    },
    'filters': {
        'package_path_filter': {
            '()': PackagePathFilter,
        },
        'runtime_filter': {
            '()': RuntimeFilter,
        }
    },
    'handlers': {
        'console': {
            'class': 'logging.StreamHandler',
            'formatter': 'default',
            'filters': ['package_path_filter', 'runtime_filter'],
            'level': 'INFO',
            'stream': 'ext://sys.stdout'
        },
    },
    'loggers': {
        '': {  # root logger
            'handlers': ['console'],
            'level': 'INFO',
            'propagate': False,
        },
        'rosys': {
            'handlers': ['console'],
            'level': 'DEBUG',
            'propagate': False,
        },
        'rosys.ota': {
            'handlers': ['console'],
            'level': 'WARN',
            'propagate': False,
        }
    },
}

logging.config.dictConfig(config)


pytest.register_assert_rewrite("tests.helper")


@pytest.fixture
async def runtime() -> Generator:
    runtime = TestRuntime()
    yield runtime
    await runtime.stop()
