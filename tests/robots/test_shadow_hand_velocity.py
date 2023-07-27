import numpy as np
import pybullet as pb
from deformable_gym.envs.bullet_simulation import BulletSimulation
from deformable_gym.robots.shadow_hand import ShadowHandVelocity
from numpy.testing import assert_almost_equal

import pytest


TEST_POS = np.array([0, 0, 1])
TEST_ORN = np.array([0, 0, 0])


@pytest.fixture
def robot():
    robot = ShadowHandVelocity(world_pos=TEST_POS, world_orn=TEST_ORN, base_commands=True)

    return robot


def test_shadow_hand_velocity_initial_pose(simulation, robot):
    simulation.add_robot(robot)

    actual_pose = np.concatenate(robot.multibody_pose.get_pose())
    expected_pose = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    assert_almost_equal(actual_pose, expected_pose)


def test_shadow_hand_velocity_motor_creation(simulation, robot, shadow_motors):

    simulation.add_robot(robot)

    found_motors = robot.motors.keys()

    # check motor creation
    for motor in shadow_motors:
        assert motor in found_motors


@pytest.mark.skip("TODO")
def test_shadow_hand_velocity_base_movement(simulation, robot):

    simulation.add_robot(robot)

    robot.perform_command(np.array([0.3, -0.2, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))

    for _ in range(50):
        simulation.step_to_trigger("time_step")

    actual_pose = np.concatenate(robot.multibody_pose.get_pose())
    expected_pose = np.array([0.3, -0.2, 1.1, 0.0, 0.0, 0.0, 1.0])
    assert_almost_equal(actual_pose, expected_pose)
