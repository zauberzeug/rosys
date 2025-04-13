import pyudev


def uid_from_device(device: pyudev.Device) -> str | None:
    parts = [
        device.get('ID_VENDOR_ID'),
        device.get('ID_MODEL_ID'),
        device.get('ID_SERIAL_SHORT') or device.get('ID_PATH') or device.get('DEVNAME'),
    ]
    return '-'.join(parts) if all(parts) else device.get('DEVNAME') or None


def is_capture_device(device: pyudev.Device) -> bool:
    try:
        # Check if the device has index attribute and if it's 0 (see https://askubuntu.com/a/1238225)
        return device.attributes.get('index') == b'0'
    except Exception:
        return False


def scan_for_connected_devices() -> set[str]:
    devices = pyudev.Context().list_devices(subsystem='video4linux')
    capture_devices = [device for device in devices if is_capture_device(device)]
    capture_uids = {uid_from_device(device) for device in capture_devices}
    return {uid for uid in capture_uids if uid is not None}


def device_nodes_from_uid(uid: str) -> set[str]:
    devices = pyudev.Context().list_devices()
    matching_devices = [device for device in devices if uid_from_device(device) == uid]
    return {device.device_node for device in matching_devices if device.device_node is not None}
