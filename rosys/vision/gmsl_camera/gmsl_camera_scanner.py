from __future__ import annotations

import re
import subprocess

# A `v4l2-ctl --list-devices` group looks like:
#   vi-output, gmsl-cam 9-0016 (platform:tegra-capture-vi:2):
#       /dev/video5
# We treat every video node backed by the Tegra capture-VI platform as a candidate GMSL sensor.
_GROUP_HEADER_REGEX = re.compile(r'^\S.*\(.*\):\s*$')
_VIDEO_NODE_REGEX = re.compile(r'^\s+/dev/video(\d+)\s*$')


def parse_sensor_ids(list_devices_output: str) -> set[int]:
    """Extract candidate Argus sensor ids from `v4l2-ctl --list-devices` output.

    NOTE: The mapping from a `/dev/videoN` index to an Argus ``sensor-id`` is not guaranteed to be
    identical on every board; the returned ids must be confirmed on the target device.
    """
    sensor_ids: set[int] = set()
    in_tegra_group = False
    for line in list_devices_output.splitlines():
        if _GROUP_HEADER_REGEX.match(line):
            in_tegra_group = 'tegra' in line.lower()
            continue
        node_match = _VIDEO_NODE_REGEX.match(line)
        if node_match is not None and in_tegra_group:
            sensor_ids.add(int(node_match.group(1)))
    return sensor_ids


def scan_for_sensor_ids() -> set[int]:
    """Run `v4l2-ctl --list-devices` and return the candidate Argus sensor ids found."""
    try:
        output = subprocess.run(['v4l2-ctl', '--list-devices'],
                                capture_output=True, text=True, check=True, timeout=10).stdout
    except (subprocess.SubprocessError, FileNotFoundError, OSError):
        return set()
    return parse_sensor_ids(output)
