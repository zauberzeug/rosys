import functools
import threading

import pyudev

_udev_lock = threading.Lock()


@functools.cache
def _udev_context() -> pyudev.Context:
    """Reuse a single context, because creating one costs several milliseconds."""
    return pyudev.Context()


def _video_devices() -> list[pyudev.Device]:
    # libudev keeps a non-atomic reference count on its context, so concurrent scans must not share it unguarded
    with _udev_lock:
        return list(_udev_context().list_devices(subsystem='video4linux'))


def uid_from_device(device: pyudev.Device) -> str | None:
    parts = [
        device.get('ID_VENDOR_ID'),
        device.get('ID_MODEL_ID'),
        device.get('ID_SERIAL_SHORT') or device.get('ID_PATH') or device.get('DEVNAME'),
    ]
    return '-'.join(parts) if all(parts) else device.get('DEVNAME') or None


def scan_for_connected_devices() -> set[str]:
    return {uid for uid in (uid_from_device(device) for device in _video_devices()) if uid is not None}


def device_nodes_from_uid(uid: str) -> set[str]:
    matching_devices = [device for device in _video_devices() if uid_from_device(device) == uid]
    return {device.device_node for device in matching_devices if device.device_node is not None}
