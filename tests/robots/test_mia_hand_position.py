import numpy as np
import pybullet as pb
from deformable_gym.envs.pybullet_tools import BulletSimulation
from deformable_gym.robots.mia_hand import MiaHandPosition
from numpy.testing import assert_almost_equal

import pytest


TEST_POS = np.array([0, 0, 1])
TEST_ORN = np.array([0, 0, 0])


@pytest.fixture
def robot():
    robot = MiaHandPosition(world_pos=TEST_POS, world_orn=TEST_ORN, base_commands=True)
    robot.set_thumb_opp(thumb_adducted=True)

    return robot


@pytest.fixture
def simulation():
    sim = BulletSimulation(mode=pb.DIRECT, verbose_dt=10000)
    sim.timing.add_subsystem("time_step", 100, None)

    return sim


def test_mia_hand_position(simulation, robot):

    actual_pose = np.concatenate(robot.multibody_pose.get_pose())
    expected_pose = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    assert_almost_equal(actual_pose, expected_pose)

    robot.perform_command(np.array([0.3, -0.2, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

    for _ in range(50):
        simulation.step_to_trigger("time_step")

    actual_pose = np.concatenate(robot.multibody_pose.get_pose())
    expected_pose = np.array([0.3, -0.2, 1.1, 0.0, 0.0, 0.0, 1.0])
    assert_almost_equal(actual_pose, expected_pose)
