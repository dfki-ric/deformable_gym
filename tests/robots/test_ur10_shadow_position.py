import numpy as np
import pybullet as pb
from deformable_gym.robots.ur10_shadow import UR10ShadowPosition
from numpy.testing import assert_almost_equal

import pytest


TEST_POS = np.array([0, 0, 1])
TEST_ORN = np.array([0, 0, 0])


@pytest.fixture
def robot():
    robot = UR10ShadowPosition()

    return robot


@pytest.mark.skip("TODO")
def test_ur10_shadow_position_creation(simulation, robot):

    simulation.add_robot(robot)

    actual_pose = np.concatenate(robot.multibody_pose.get_pose())
    expected_pose = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    assert_almost_equal(actual_pose, expected_pose)
