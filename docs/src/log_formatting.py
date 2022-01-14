#!/usr/bin/env python3
from nicegui import ui
import logging
import logging.config
import os
import sys
import rosys
import rosys.ui
from rosys.world import Mode, World


class PackagePathFilter(logging.Filter):
    '''Provides relative path for log formatter.
    Original code borrowed from https://stackoverflow.com/a/52582536/3419103
    '''

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


logging.config.dictConfig({
    'version': 1,
    'disable_existing_loggers': True,  # to make sure this config is used
    'formatters': {
        'default': {
            'format': '%(asctime)s.%(msecs)03d [%(levelname)s] %(relativepath)s:%(lineno)d: %(message)s',
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
            'level': 'DEBUG',
            'propagate': False,
        },
    },
})

# setup
runtime = rosys.Runtime(world=World(mode=Mode.SIMULATION))
rosys.ui.configure(ui, runtime)

rosys.ui.joystick()

ui.run(title='RoSys', port=8080)
