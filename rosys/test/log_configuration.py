import logging
import logging.config
import os
import sys

import coloredlogs
import icecream
import rosys


class PackagePathFilter(logging.Filter):

    # https://stackoverflow.com/a/52582536/3419103
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


class RosysFilter(logging.Filter):

    def filter(self, record: logging.LogRecord) -> bool:
        from rosys.test.helpers import odometer as odo
        if odo:
            record.rosys_time = rosys.time()
            record.robot_pose = f'{odo.prediction.x:.2f}, {odo.prediction.y:1.2f}, {odo.prediction.yaw_deg:1.2f}'
        else:
            record.rosys_time = 0
            record.robot_pose = 'no robot pose yet'
        return True


def setup() -> None:
    icecream.install()

    config = {
        'version': 1,
        'disable_existing_loggers': True,
        'formatters': {
            'default': {
                '()': coloredlogs.ColoredFormatter,
                'format': '%(rosys_time).2f [%(levelname)s] %(robot_pose)s %(relative_path)s:%(lineno)d: %(message)s',
                'datefmt': '%Y-%m-%d %H:%M:%S',
            },
        },
        'filters': {
            'package_path_filter': {
                '()': PackagePathFilter,
            },
            'rosys_filter': {
                '()': RosysFilter,
            }
        },
        'handlers': {
            'console': {
                'class': 'logging.StreamHandler',
                'formatter': 'default',
                'filters': ['package_path_filter', 'rosys_filter'],
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
