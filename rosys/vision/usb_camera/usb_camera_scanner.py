from typing import Optional

import pyudev


def uid_from_device(device) -> Optional[str]:
    vendor_id = device.get('ID_VENDOR_ID')
    model_id = device.get('ID_MODEL_ID')
    serial_short = device.get('ID_SERIAL_SHORT')

    if vendor_id is not None and model_id is not None and serial_short is not None:
        uid = '-'.join([vendor_id, model_id, serial_short])
        return uid

    return None


def scan_for_connected_devices() -> set[str]:
    # Create a context for working with udev
    context = pyudev.Context()

    # List all devices connected to the system
    udev_devices = list(context.list_devices())

    video_device_ids = set()
    for device in udev_devices:
        if device.subsystem == 'video4linux':
            uid = uid_from_device(device)
            if uid is not None:
                video_device_ids.add(uid)

    return video_device_ids


def device_nodes_from_uid(uid: str) -> set[str]:

    context = pyudev.Context()

    # List all devices connected to the system
    udev_devices = list(context.list_devices())

    node_paths = set()

    for device in udev_devices:
        dev_uid = uid_from_device(device)

        if dev_uid == uid:
            if device.device_node is not None:
                node_paths.add(device.device_node)
    return node_paths
