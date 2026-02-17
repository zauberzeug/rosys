"""Hardware acceleration detection and GStreamer pipeline configuration for RTSP cameras.

This module provides utilities to detect NVIDIA Jetson hardware and configure
GStreamer pipelines to use hardware-accelerated video decoding when available.
"""
from __future__ import annotations

import logging
import os
import shutil
import subprocess
from dataclasses import dataclass
from enum import Enum, auto
from functools import lru_cache
from typing import Literal

log = logging.getLogger('rosys.vision.rtsp_camera.hardware_acceleration')


class JetsonModel(Enum):
    """Known Jetson hardware models."""
    ORIN_NANO = auto()
    ORIN_NX = auto()
    ORIN_AGX = auto()
    XAVIER_NX = auto()
    XAVIER_AGX = auto()
    NANO = auto()
    TX2 = auto()
    UNKNOWN = auto()


@dataclass
class HardwareCapabilities:
    """Detected hardware capabilities for video processing."""
    is_jetson: bool
    jetson_model: JetsonModel | None
    has_nvv4l2decoder: bool
    has_nvvidconv: bool
    has_nvjpegenc: bool
    l4t_version: str | None


@lru_cache(maxsize=1)
def detect_jetson_hardware() -> HardwareCapabilities:
    """Detect if running on NVIDIA Jetson hardware and available acceleration features.

    Returns cached result after first call.
    """
    is_jetson = False
    jetson_model = None
    l4t_version = None

    # Check for Jetson by looking at /etc/nv_tegra_release or device tree
    try:
        if os.path.exists('/etc/nv_tegra_release'):
            is_jetson = True
            with open('/etc/nv_tegra_release') as f:
                content = f.read()
                # Parse L4T version from content like "# R35 (release), REVISION: 4.1"
                if 'R' in content:
                    parts = content.split()
                    for part in parts:
                        if part.startswith('R'):
                            l4t_version = part
                            break

        # Alternative detection via device tree
        if not is_jetson and os.path.exists('/proc/device-tree/compatible'):
            with open('/proc/device-tree/compatible', 'rb') as f:
                compatible = f.read().decode('utf-8', errors='ignore').lower()
                if 'nvidia' in compatible and 'tegra' in compatible:
                    is_jetson = True

        # Detect specific Jetson model
        if is_jetson:
            jetson_model = _detect_jetson_model()

    except (OSError, PermissionError) as e:
        log.debug('Could not detect Jetson hardware: %s', e)

    # Check for GStreamer NVIDIA plugins
    has_nvv4l2decoder = _check_gst_element('nvv4l2decoder')
    has_nvvidconv = _check_gst_element('nvvidconv')
    has_nvjpegenc = _check_gst_element('nvjpegenc')

    capabilities = HardwareCapabilities(
        is_jetson=is_jetson,
        jetson_model=jetson_model,
        has_nvv4l2decoder=has_nvv4l2decoder,
        has_nvvidconv=has_nvvidconv,
        has_nvjpegenc=has_nvjpegenc,
        l4t_version=l4t_version,
    )

    log.info('Hardware capabilities detected: %s', capabilities)
    return capabilities


def _detect_jetson_model() -> JetsonModel:
    """Detect specific Jetson model from device tree or tegrastats."""
    try:
        # Try reading from device tree model
        if os.path.exists('/proc/device-tree/model'):
            with open('/proc/device-tree/model', 'rb') as f:
                model_str = f.read().decode('utf-8', errors='ignore').lower()

                if 'orin' in model_str:
                    if 'nano' in model_str:
                        return JetsonModel.ORIN_NANO
                    if 'nx' in model_str:
                        return JetsonModel.ORIN_NX
                    return JetsonModel.ORIN_AGX
                if 'xavier' in model_str:
                    if 'nx' in model_str:
                        return JetsonModel.XAVIER_NX
                    return JetsonModel.XAVIER_AGX
                if 'nano' in model_str:
                    return JetsonModel.NANO
                if 'tx2' in model_str:
                    return JetsonModel.TX2

    except (OSError, PermissionError):
        pass

    return JetsonModel.UNKNOWN


def _check_gst_element(element_name: str) -> bool:
    """Check if a GStreamer element is available."""
    if not shutil.which('gst-inspect-1.0'):
        return False

    try:
        result = subprocess.run(
            ['gst-inspect-1.0', element_name],
            check=False, capture_output=True,
            timeout=5,
        )
        return result.returncode == 0
    except (subprocess.TimeoutExpired, OSError):
        return False


def build_decoder_pipeline(
    codec: Literal['h264', 'h265'],
    *,
    use_hardware: bool = True,
    output_format: str = 'RGB',
) -> str:
    """Build the GStreamer decoder pipeline string for the given codec.

    Args:
        codec: Video codec ('h264' or 'h265')
        use_hardware: Whether to attempt hardware acceleration
        output_format: Output pixel format (default: RGB)

    Returns:
        GStreamer pipeline elements string (without source and sink)
    """
    caps = detect_jetson_hardware()

    if use_hardware and caps.has_nvv4l2decoder and caps.has_nvvidconv:
        # NVIDIA Jetson hardware-accelerated pipeline
        # nvv4l2decoder outputs NV12 to NVMM memory, nvvidconv converts to RGB
        log.debug('Using NVIDIA hardware decoder for %s', codec)
        return (
            f'rtp{codec}depay ! '
            f'nvv4l2decoder ! '
            f'nvvidconv ! '
            f'video/x-raw,format={output_format}'
        )

    # Software decoding fallback using libav
    log.debug('Using software decoder (avdec) for %s', codec)
    return (
        f'rtp{codec}depay ! '
        f'avdec_{codec} ! '
        f'videoconvert ! '
        f'video/x-raw,format={output_format}'
    )


def build_jpeg_encoder_pipeline(*, use_hardware: bool = True) -> str:
    """Build the GStreamer JPEG encoder pipeline string.

    Args:
        use_hardware: Whether to attempt hardware acceleration

    Returns:
        GStreamer pipeline elements for JPEG encoding
    """
    caps = detect_jetson_hardware()

    if use_hardware and caps.has_nvjpegenc and caps.has_nvvidconv:
        # NVIDIA hardware JPEG encoding
        log.debug('Using NVIDIA hardware JPEG encoder')
        return 'nvvidconv ! nvjpegenc'

    # Software fallback
    log.debug('Using software JPEG encoder')
    return 'jpegenc'


def get_hardware_info_string() -> str:
    """Get a human-readable string describing detected hardware capabilities."""
    caps = detect_jetson_hardware()

    if not caps.is_jetson:
        return 'Non-Jetson hardware (software decoding)'

    model_name = caps.jetson_model.name if caps.jetson_model else 'Unknown'
    l4t = caps.l4t_version or 'unknown'

    features = []
    if caps.has_nvv4l2decoder:
        features.append('HW decoder')
    if caps.has_nvvidconv:
        features.append('HW converter')
    if caps.has_nvjpegenc:
        features.append('HW JPEG')

    features_str = ', '.join(features) if features else 'no HW features'
    return f'Jetson {model_name} (L4T {l4t}): {features_str}'
