#!/usr/bin/env python3
import logging
import logging.config
import os.path
import sys
from collections.abc import Callable
from typing import cast

from nicegui import ui

from rosys.driving import Odometer, Steerer, joystick
from rosys.hardware import RobotSimulation, WheelsSimulation


class PackagePathFilter(logging.Filter):
    """Provides relative path for log formatter.

    Original code borrowed from https://stackoverflow.com/a/52582536/3419103
    """

    def filter(self, record: logging.LogRecord) -> bool:
        pathname = record.pathname
        record.relative_path = None
        abs_sys_paths = map(cast(Callable[[str], str], os.path.abspath), sys.path)
        for path in sorted(abs_sys_paths, key=len, reverse=True):  # longer paths first
            path_ = path if path.endswith(os.sep) else path + os.sep
            if pathname.startswith(path_):
                record.relative_path = os.path.relpath(pathname, path_)
                break
        return True


logging.config.dictConfig({
    'version': 1,
    'disable_existing_loggers': True,  # to make sure this config is used
    'formatters': {
        'default': {
            'format': '%(asctime)s.%(msecs)03d [%(levelname)s] %(relative_path)s:%(lineno)d: %(message)s',
            'datefmt': '%Y-%m-%d %H:%M:%S',
        },
    },
    'filters': {
        'package_path_filter': {
            '()': PackagePathFilter,
        },
    },
    'handlers': {
        'console': {
            'class': 'logging.StreamHandler',
            'filters': ['package_path_filter'],
            'formatter': 'default',
            'level': 'DEBUG',
            'stream': 'ext://sys.stdout'
        },
    },
    'loggers': {
        '': {  # this root logger is used for everything without a specific logger
            'handlers': ['console'],
            'level': 'WARN',
            'propagate': False,
        },
        'rosys': {
            'handlers': ['console'],
            'level': 'INFO',
            'propagate': False,
        },
    },
})

wheels = WheelsSimulation()
steerer = Steerer(wheels)
odometer = Odometer(wheels)
robot = RobotSimulation([wheels])

joystick(steerer)

ui.run(title='RoSys')
