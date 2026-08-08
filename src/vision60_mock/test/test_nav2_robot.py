import math

from vision60_mock.nav2_robot import yaw_to_quaternion


def test_yaw_to_quaternion_for_pi():
    _, _, z, w = yaw_to_quaternion(math.pi)
    assert math.isclose(z, 1.0)
    assert math.isclose(w, 0.0, abs_tol=1e-12)
