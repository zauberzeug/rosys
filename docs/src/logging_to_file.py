#!/usr/bin/env python3
from nicegui import ui
import logging
import logging.config
import os
import rosys
import rosys.ui
from rosys.world import Mode, World

logging.config.dictConfig({
    'version': 1,
    'disable_existing_loggers': True,  # to make sure this config is used
    'formatters': {
        'default': {
            'format': '%(asctime)s - %(levelname)s - %(message)s',
            'datefmt': '%Y-%m-%d %H:%M:%S',
        },
    },
    'handlers': {
        'console': {
            'class': 'logging.StreamHandler',
            'formatter': 'default',
            'level': 'DEBUG',
            'stream': 'ext://sys.stdout'
        },
        'file': {
            'level': 'DEBUG',
            'class': 'logging.handlers.RotatingFileHandler',
            'formatter': 'default',
            'filename': os.path.expanduser('~/.rosys/example.log'),
            'maxBytes': 1024,
            'backupCount': 3
        }
    },
    'loggers': {
        '': {  # this root logger is used for everything without a specific logger
            'handlers': ['console', 'file'],
            'level': 'WARN',
            'propagate': False,
        },
        'rosys': {
            'handlers': ['console', 'file'],
            'level': 'INFO',
            'propagate': False,
        },
        'rosys.event': {
            'handlers': ['file'],
            'level': 'DEBUG',
            'propagate': False,
        },
        'rosys.runtime': {
            'handlers': ['file'],
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
