import logging
import logging.config

import coloredlogs

import rosys
from rosys.helpers import PackagePathFilter


class RosysFilter(logging.Filter):

    def filter(self, record: logging.LogRecord) -> bool:
        from rosys.testing.helpers import odometer as odo  # pylint: disable=import-outside-toplevel
        if odo:
            record.rosys_time = rosys.time()
            record.robot_pose = f'{odo.prediction.x:.2f}, {odo.prediction.y:1.2f}, {odo.prediction.yaw_deg:1.2f}'
        else:
            record.rosys_time = 0
            record.robot_pose = 'no robot pose yet'
        return True


def setup() -> None:
    config = {
        'version': 1,
        'disable_existing_loggers': True,
        'formatters': {
            'default': {
                '()': coloredlogs.ColoredFormatter,
                'format': r'%(rosys_time).2f [%(levelname)s] %(robot_pose)s %(relative_path)s:%(lineno)d: %(message)s',
                'datefmt': r'%Y-%m-%d %H:%M:%S',
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
