from typing import Generator
import pytest
import logging
import logging.config
import os
import sys
from .runtime import TestRuntime

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


class WorldTimeFilter(logging.Filter):
    def filter(self, record):
        from .helper import global_runtime
        if global_runtime:
            #record.worldtime = str(round(global_runtime.world.time, 2))
            record.worldtime = global_runtime.world.time
        else:
            record.worldtime = 0
        return True


config = {
    'version': 1,
    'disable_existing_loggers': True,
    'formatters': {
        'default': {
            'format': '%(worldtime).2f [%(levelname)s] %(relativepath)s:%(lineno)d: %(message)s',
            'datefmt': '%Y-%m-%d %H:%M:%S',
        },
    },
    'filters': {
        'package_path_filter': {
            '()': PackagePathFilter,
        },
        'world_time_filter': {
            '()': WorldTimeFilter,
        }
    },
    'handlers': {
        'console': {
            'class': 'logging.StreamHandler',
            'formatter': 'default',
            'filters': ['package_path_filter', 'world_time_filter'],
            'level': 'INFO',
            'stream': 'ext://sys.stdout'
        },
    },
    'loggers': {
        '': {  # root logger
            'handlers': ['console'],
            'level': 'WARN',
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
