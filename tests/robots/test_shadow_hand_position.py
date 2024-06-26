import numpy as np
import pybullet as pb
import pytest
from numpy.testing import assert_almost_equal

from deformable_gym.robots.shadow_hand import ShadowHandPosition

TEST_POS = np.array([0, 0, 1])
TEST_ORN = pb.getQuaternionFromEuler(np.array([0, 0, 0]))


@pytest.fixture
def robot(simulation):
    robot = ShadowHandPosition(
        pb_client=simulation.pb_client,
        world_pos=TEST_POS,
        world_orn=TEST_ORN,
        base_commands=True,
    )

    return robot


def test_shadow_hand_position_initial_pose(simulation, robot, shadow_motors):

    simulation.add_robot(robot)

    actual_pose = np.concatenate(robot.multibody_pose.get_pose())
    expected_pose = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    assert_almost_equal(actual_pose, expected_pose)

    found_motors = robot.motors.keys()

    # check motor creation
    for motor in shadow_motors:
        assert motor in found_motors


def test_shadow_hand_position_motor_creation(simulation, robot, shadow_motors):

    # check motor creation
    assert set(robot.motors.keys()) == set(shadow_motors)


@pytest.mark.skip("TODO")
def test_shadow_hand_position_base_movement(simulation, robot):
    simulation.add_robot(robot)

    robot.perform_command(
        np.array(
            [
                0.3,
                -0.2,
                0.1,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            ]
        )
    )

    for _ in range(50):
        simulation.step_to_trigger("time_step")

    actual_pose = np.concatenate(robot.multibody_pose.get_pose())
    expected_pose = np.array([0.3, -0.2, 1.1, 0.0, 0.0, 0.0, 1.0])
    assert_almost_equal(actual_pose, expected_pose)
