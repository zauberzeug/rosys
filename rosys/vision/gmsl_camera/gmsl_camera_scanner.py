from __future__ import annotations

import re
import subprocess

# A `v4l2-ctl --list-devices` group looks like:
#   vi-output, gmsl-cam 0-0015 (platform:tegra-capture-vi:2):
#       /dev/video4
#       /dev/video5
# Each Tegra capture-VI group backed by ≥1 video node is one GMSL sensor channel (a sensor typically
# exposes several `/dev/video` nodes, so we count channels, not nodes). Argus enumerates the cameras
# as sensor-ids 0..N-1, which is the id `nvarguscamerasrc sensor-id=` expects -- unrelated to the
# `/dev/videoN` index.
_GROUP_HEADER_REGEX = re.compile(r'^\S.*\(.*\):\s*$')
_VIDEO_NODE_REGEX = re.compile(r'^\s+/dev/video\d+\s*$')


def count_gmsl_sensors(list_devices_output: str) -> int:
    """Count the Tegra capture-VI sensor channels (each with ≥1 video node) in `v4l2-ctl --list-devices` output."""
    count = 0
    in_tegra_group = False
    group_has_video = False
    for line in list_devices_output.splitlines():
        if _GROUP_HEADER_REGEX.match(line):
            if in_tegra_group and group_has_video:
                count += 1
            in_tegra_group = 'tegra-capture-vi' in line.lower()
            group_has_video = False
        elif in_tegra_group and _VIDEO_NODE_REGEX.match(line):
            group_has_video = True
    if in_tegra_group and group_has_video:
        count += 1
    return count


def parse_sensor_ids(list_devices_output: str) -> set[int]:
    """Return candidate Argus sensor ids (``0..N-1``) for the GMSL sensors in `v4l2-ctl --list-devices` output.

    NOTE: The order of Argus sensor-ids relative to the capture-VI channels must be confirmed on the
    target device; this assumes Argus enumerates them contiguously from 0.
    """
    return set(range(count_gmsl_sensors(list_devices_output)))


def scan_for_sensor_ids() -> set[int]:
    """Run `v4l2-ctl --list-devices` and return the candidate Argus sensor ids found."""
    try:
        output = subprocess.run(['v4l2-ctl', '--list-devices'],
                                capture_output=True, text=True, check=True, timeout=10).stdout
    except (subprocess.SubprocessError, FileNotFoundError, OSError):
        return set()
    return parse_sensor_ids(output)
