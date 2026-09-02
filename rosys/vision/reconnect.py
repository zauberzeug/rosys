import logging

MIN_RECONNECT_INTERVAL = 0.1
"""Shortest wait between two connection attempts.

An interval of zero reads as "retry at once", which on a single event loop means a capture loop
that never lets anything else run.
"""

MAX_RECONNECT_INTERVAL = 30.0
"""Wait between attempts after a camera answered without a stream, so a camera that starts
answering again is picked up within this long at the latest."""


def clamp_reconnect_interval(interval: float, log: logging.Logger) -> float:
    """Hold `interval` to `MIN_RECONNECT_INTERVAL`, saying so when the requested value cannot be honored."""
    if interval >= MIN_RECONNECT_INTERVAL:
        return interval
    log.warning('a reconnect interval of %.2f s is too short; using %.2f s', interval, MIN_RECONNECT_INTERVAL)
    return MIN_RECONNECT_INTERVAL
