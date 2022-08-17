#!/usr/bin/env python3
import logging
import logging.config
import os
import sys

from nicegui import ui
from rosys.driving import Joystick, Odometer, Steerer
from rosys.hardware import WheelsSimulation


class PackagePathFilter(logging.Filter):
    '''Provides relative path for log formatter.
    Original code borrowed from https://stackoverflow.com/a/52582536/3419103
    '''

    def filter(self, record: logging.LogRecord) -> bool:
        pathname = record.pathname
        record.relative_path = None
        abs_sys_paths = map(os.path.abspath, sys.path)
        for path in sorted(abs_sys_paths, key=len, reverse=True):  # longer paths first
            if not path.endswith(os.sep):
                path += os.sep
            if pathname.startswith(path):
                record.relative_path = os.path.relpath(pathname, path)
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

# setup
wheels = WheelsSimulation()
steerer = Steerer(wheels)
odometer = Odometer(wheels)

# ui
Joystick(steerer)

# start
ui.run(title='RoSys')
