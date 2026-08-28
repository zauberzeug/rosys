import logging
import random

MIN_RECONNECT_INTERVAL = 0.1
"""Shortest wait between two connection attempts.

An interval of zero reads as "retry at once", which on a single event loop means a capture loop
that never lets anything else run.
"""

MAX_RECONNECT_INTERVAL = 30.0
"""Longest wait between two connection attempts, i.e. the cap of the back-off.

It also caps the wait after a camera refused the stream, so a camera that starts answering again is
picked up within this long at the latest.
"""

RECONNECT_JITTER = 0.2
"""Relative spread of the wait between two connection attempts."""


def retry_delay(reconnect_interval: float, failed_attempts: int = 0) -> float:
    """How long a device waits before its next connection attempt.

    The wait doubles from the second consecutive failure on, up to `MAX_RECONNECT_INTERVAL`, so an
    unreachable camera stops hammering, and is jittered so cameras that went down together do not
    retry in lock-step.

    :param reconnect_interval: the configured wait after a stream that was working ended
    :param failed_attempts: how many attempts in a row have failed to deliver a single frame
    """
    interval = max(reconnect_interval, MIN_RECONNECT_INTERVAL)
    interval *= 2 ** min(max(failed_attempts - 1, 0), 16)
    interval *= random.uniform(1 - RECONNECT_JITTER, 1 + RECONNECT_JITTER)
    return min(max(interval, MIN_RECONNECT_INTERVAL), MAX_RECONNECT_INTERVAL)


def clamp_reconnect_interval(interval: float, log: logging.Logger) -> float:
    """Hold `interval` to `MIN_RECONNECT_INTERVAL`, saying so when the requested value cannot be honored."""
    if interval >= MIN_RECONNECT_INTERVAL:
        return interval
    log.warning('a reconnect interval of %.2f s is too short; using %.2f s', interval, MIN_RECONNECT_INTERVAL)
    return MIN_RECONNECT_INTERVAL
