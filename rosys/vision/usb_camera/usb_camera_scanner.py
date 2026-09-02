import functools
import threading

import pyudev

_udev_lock = threading.Lock()


@functools.cache
def _udev_context() -> pyudev.Context:
    """Reuse a single context, because creating one costs several milliseconds."""
    return pyudev.Context()


def uid_from_device(device: pyudev.Device) -> str | None:
    parts = [
        device.get('ID_VENDOR_ID'),
        device.get('ID_MODEL_ID'),
        device.get('ID_SERIAL_SHORT') or device.get('ID_PATH') or device.get('DEVNAME'),
    ]
    return '-'.join(parts) if all(parts) else device.get('DEVNAME') or None


def _scan() -> list[tuple[str | None, str | None]]:
    """List ``(uid, device node)`` of all video devices.

    libudev keeps a non-atomic reference count on its context, so concurrent scans must not share it
    unguarded. `pyudev.Device` objects hold a reference to that context and drop it when they are
    garbage-collected, so none of them may leave the lock; only plain strings do.
    """
    with _udev_lock:
        return [(uid_from_device(device), device.device_node)
                for device in _udev_context().list_devices(subsystem='video4linux')]


def scan_for_connected_devices() -> set[str]:
    return {uid for uid, _ in _scan() if uid is not None}


def device_nodes_from_uid(uid: str) -> set[str]:
    return {node for device_uid, node in _scan() if device_uid == uid and node is not None}
