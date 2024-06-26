import numpy as np
import pytest
from numpy.testing import assert_almost_equal

from deformable_gym.robots.ur5_mia import UR5MiaPosition


@pytest.fixture
def robot(simulation):
    robot = UR5MiaPosition(pb_client=simulation.pb_client)

    return robot


@pytest.mark.skip("TODO")
def test_ur5_mia_position_initial_position(simulation, robot):

    actual_pose = np.concatenate(robot.multibody_pose.get_pose())
    expected_pose = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    assert_almost_equal(actual_pose, expected_pose)


def test_ur5_mia_position_creation(
    simulation, robot, ur5_mia_motors, ur5_mia_sensors
):

    # check motor creation
    assert set(robot.motors.keys()) == set(ur5_mia_motors)
    # check sensor creation
    assert set(robot.sensors.keys()) == set(ur5_mia_sensors)
