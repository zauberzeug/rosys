import logging

from nicegui import ui

LEVELS = {
    logging.NOTSET: 'NOTSET',
    logging.DEBUG: 'DEBUG',
    logging.INFO: 'INFO',
    logging.WARNING: 'WARNING',
    logging.ERROR: 'ERROR',
    logging.CRITICAL: 'CRITICAL',
}


class LoggingPage:
    """Logging Page

    This module creates a page to change the log levels of different loggers.
    A list of logger names like `["field_friend", "rosys"]` can be passed to group them together.
    It is mounted at /logging.
    """

    def __init__(self, group_names: list[str] | None = None) -> None:
        group_names = group_names or []

        @ui.page('/logging')
        def page():
            loggers = [logging.getLogger(name) for name in logging.root.manager.loggerDict]  # pylint: disable=no-member
            groups = {name: [l for l in loggers if l.name.startswith(name)] for name in group_names}
            groups['others'] = [l for l in loggers if not l.name.startswith(tuple(group_names))]

            @ui.refreshable
            def _column(group_name: str, loggers: list[logging.Logger]) -> None:
                with ui.column():
                    ui.label(group_name).classes('text-xl')
                    for logger in sorted(loggers, key=lambda logger: logger.name.lower()):
                        ui.select(LEVELS, label=logger.name, value=logger.getEffectiveLevel(),
                                  on_change=lambda e, logger=logger: _update_level(logger, e.value)) \
                            .classes('w-64').props('dense outlined')

            def _update_level(logger: logging.Logger, level: int) -> None:
                logger.setLevel(level)
                _column.refresh()

            with ui.row():
                for name, loggers in groups.items():
                    _column(name, loggers)
