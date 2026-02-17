"""Tests for hardware acceleration detection and pipeline building."""

from rosys.vision.rtsp_camera.hardware_acceleration import (
    HardwareCapabilities,
    JetsonModel,
    build_decoder_pipeline,
    build_jpeg_encoder_pipeline,
    detect_jetson_hardware,
    get_hardware_info_string,
)


def test_detect_jetson_hardware_returns_capabilities():
    """Test that hardware detection returns a valid capabilities object."""
    caps = detect_jetson_hardware()
    assert isinstance(caps, HardwareCapabilities)
    assert isinstance(caps.is_jetson, bool)
    assert caps.has_nvv4l2decoder is not None
    assert caps.has_nvvidconv is not None
    assert caps.has_nvjpegenc is not None


def test_detect_jetson_hardware_is_cached():
    """Test that hardware detection result is cached."""
    caps1 = detect_jetson_hardware()
    caps2 = detect_jetson_hardware()
    assert caps1 is caps2


def test_build_decoder_pipeline_h264_software():
    """Test building H264 software decoder pipeline."""
    pipeline = build_decoder_pipeline('h264', use_hardware=False)
    assert 'rtph264depay' in pipeline
    assert 'avdec_h264' in pipeline
    assert 'videoconvert' in pipeline
    assert 'video/x-raw,format=RGB' in pipeline


def test_build_decoder_pipeline_h265_software():
    """Test building H265 software decoder pipeline."""
    pipeline = build_decoder_pipeline('h265', use_hardware=False)
    assert 'rtph265depay' in pipeline
    assert 'avdec_h265' in pipeline
    assert 'videoconvert' in pipeline
    assert 'video/x-raw,format=RGB' in pipeline


def test_build_decoder_pipeline_custom_format():
    """Test building pipeline with custom output format."""
    pipeline = build_decoder_pipeline('h264', use_hardware=False, output_format='BGR')
    assert 'video/x-raw,format=BGR' in pipeline


def test_build_jpeg_encoder_pipeline_software():
    """Test building software JPEG encoder pipeline."""
    pipeline = build_jpeg_encoder_pipeline(use_hardware=False)
    assert 'jpegenc' in pipeline


def test_get_hardware_info_string():
    """Test that hardware info string is human readable."""
    info = get_hardware_info_string()
    assert isinstance(info, str)
    assert len(info) > 0


def test_jetson_model_enum():
    """Test JetsonModel enum values exist."""
    assert JetsonModel.ORIN_NANO is not None
    assert JetsonModel.ORIN_NX is not None
    assert JetsonModel.ORIN_AGX is not None
    assert JetsonModel.XAVIER_NX is not None
    assert JetsonModel.XAVIER_AGX is not None
    assert JetsonModel.NANO is not None
    assert JetsonModel.UNKNOWN is not None
