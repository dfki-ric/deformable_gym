import numpy as np
import pybullet as pb
from deformable_gym.robots.ur5_mia import UR5MiaPosition
from numpy.testing import assert_almost_equal

import pytest


TEST_POS = np.array([0, 0, 1])
TEST_ORN = pb.getQuaternionFromEuler(np.array([0, 0, 0]))


@pytest.fixture
def robot():
    robot = UR5MiaPosition()

    return robot


@pytest.mark.skip("TODO")
def test_ur5_mia_position_initial_position(simulation, robot):

    actual_pose = np.concatenate(robot.multibody_pose.get_pose())
    expected_pose = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    assert_almost_equal(actual_pose, expected_pose)


def test_ur5_mia_position_creation(simulation, robot, ur5_mia_motors, ur5_mia_sensors):

    # check motor creation
    assert set(robot.motors.keys()) == set(ur5_mia_motors)
    # check sensor creation
    assert set(robot.sensors.keys()) == set(ur5_mia_sensors)
