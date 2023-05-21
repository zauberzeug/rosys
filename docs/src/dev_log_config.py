#!/usr/bin/env python3
import logging
import logging.config

from nicegui import ui

from rosys.driving import Odometer, Steerer, joystick
from rosys.hardware import RobotSimulation, WheelsSimulation

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
