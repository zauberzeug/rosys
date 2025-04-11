#!/usr/bin/env python3
import logging
import logging.config
from pathlib import Path

from nicegui import ui

from rosys.driving import Odometer, Steerer, joystick
from rosys.hardware import RobotSimulation, WheelsSimulation

PATH = Path('~/.rosys').expanduser()
PATH.mkdir(parents=True, exist_ok=True)

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
            'filename': PATH / 'example.log',
            'maxBytes': 1024 * 1000,
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
        'rosys.core': {
            'handlers': ['file'],
            'level': 'DEBUG',
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
