import logging

MIN_RECONNECT_INTERVAL = 0.1
"""Shortest wait between two connection attempts; zero would starve the event loop."""

MAX_RECONNECT_INTERVAL = 30.0
"""Longest wait between two connection attempts, so a camera that answers again is picked up in time."""


def clamp_reconnect_interval(interval: float, log: logging.Logger) -> float:
    """Hold `interval` to `MIN_RECONNECT_INTERVAL`, saying so when the requested value cannot be honored."""
    if interval >= MIN_RECONNECT_INTERVAL:
        return interval
    log.warning('a reconnect interval of %.2f s is too short; using %.2f s', interval, MIN_RECONNECT_INTERVAL)
    return MIN_RECONNECT_INTERVAL
