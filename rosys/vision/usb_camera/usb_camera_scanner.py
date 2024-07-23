import pyudev


def uid_from_device(device: pyudev.Device) -> str | None:
    parts = [
        device.get('ID_VENDOR_ID'),
        device.get('ID_MODEL_ID'),
        device.get('ID_SERIAL_SHORT') or device.get('ID_PATH') or device.get('DEVNAME'),
    ]
    return '-'.join(parts) if all(parts) else device.get('DEVNAME') or None


def scan_for_connected_devices() -> set[str]:
    devices = pyudev.Context().list_devices()
    video_device_ids = {uid_from_device(device) for device in devices if device.subsystem == 'video4linux'}
    return {uid for uid in video_device_ids if uid is not None}


def device_nodes_from_uid(uid: str) -> set[str]:
    devices = pyudev.Context().list_devices()
    matching_devices = [device for device in devices if uid_from_device(device) == uid]
    return {device.device_node for device in matching_devices if device.device_node is not None}
