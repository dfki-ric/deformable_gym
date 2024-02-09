import numpy as np
import pybullet as pb
from deformable_gym.robots.ur5_mia import UR5MiaVelocity
from numpy.testing import assert_almost_equal

import pytest


TEST_POS = np.array([0, 0, 1])
TEST_ORN = pb.getQuaternionFromEuler(np.array([0, 0, 0]))


@pytest.fixture
def robot():
    robot = UR5MiaVelocity()

    return robot


@pytest.mark.skip("TODO")
def test_ur5_mia_velocity_creation(simulation, robot):

    actual_pose = np.concatenate(robot.multibody_pose.get_pose())
    expected_pose = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    assert_almost_equal(actual_pose, expected_pose)

    robot.perform_command(np.array([0.3, -0.2, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))


def test_ur5_mia_velocity_motor_creation(simulation, robot, ur5_mia_motors, ur5_mia_sensors):
    # check motor creation
    assert set(robot.motors.keys()) == set(ur5_mia_motors)
    # check sensor creation
    assert set(robot.sensors.keys()) == set(ur5_mia_sensors)

