import numpy as np

from rosys.vision import Image, SimulatedCamera
from rosys.vision.image_route import _try_get_jpeg


async def test_route_honors_requested_compression(rosys_integration):
    """The route encodes at the requested quality, also on the fast path for an unmodified image."""
    camera = SimulatedCamera(id='test_cam', width=320, height=240, fps=1)
    noise = np.random.default_rng(seed=0).integers(0, 256, size=(240, 320, 3), dtype=np.uint8)
    camera.images.append(Image.from_array(noise, camera_id=camera.id, time=1.0))
    request = {'shrink': 1.0, 'max_dimension': None, 'undistort': False, 'fast': True}

    default_jpeg = await _try_get_jpeg(camera, '1.0', compression=90, **request)
    low_quality_jpeg = await _try_get_jpeg(camera, '1.0', compression=25, **request)

    assert default_jpeg is not None
    assert low_quality_jpeg is not None
    assert len(low_quality_jpeg) < len(default_jpeg) / 2
