from typing import Generator
from runtime import Runtime, Mode
import pytest
pytest.register_assert_rewrite("tests.helper")


@pytest.fixture
def runtime() -> Generator:

    runtime = Runtime(Mode.TEST)

    from tests.helper import set_global_runtime
    set_global_runtime(runtime)

    yield runtime
