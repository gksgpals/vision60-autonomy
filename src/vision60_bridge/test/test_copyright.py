from ament_copyright.main import main
import pytest


@pytest.mark.skip(reason='Copyright headers are not added yet.')
@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    assert main(argv=['.', 'test']) == 0
