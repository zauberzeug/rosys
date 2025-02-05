from rosys.testing.fixtures import *  # pylint: disable=wildcard-import,unused-wildcard-import # noqa: F403


@pytest.fixture
def camera_parameters() -> dict:
    return {
        'id': 'test_cam',
        'name': 'T3:5T',
        'connect_after_init': False,
        'base_path_overwrite': 'new_base_path',
        'image_history_length': 10
    }
