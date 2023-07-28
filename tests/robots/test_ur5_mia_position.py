import numpy as np
from deformable_gym.robots.ur5_mia import UR5MiaPosition
from numpy.testing import assert_almost_equal

import pytest


TEST_POS = np.array([0, 0, 1])
TEST_ORN = np.array([0, 0, 0])


@pytest.fixture
def robot():
    robot = UR5MiaPosition()

    return robot


@pytest.mark.skip("TODO")
def test_ur5_mia_position_initial_position(simulation, robot):

    actual_pose = np.concatenate(robot.multibody_pose.get_pose())
    expected_pose = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    assert_almost_equal(actual_pose, expected_pose)


def test_mia_hand_position_creation(simulation, robot, mia_motors, mia_sensors):

    found_motors = robot.motors.keys()

    # check motor creation
    for motor in mia_motors:
        assert motor in found_motors

    found_sensors = robot.sensors.keys()

    # check sensor creation
    for sensor in mia_sensors:
        assert sensor in found_sensors

