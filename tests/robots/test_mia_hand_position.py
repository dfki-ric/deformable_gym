import numpy as np
import pybullet as pb
import pytest
from numpy.testing import assert_almost_equal

from deformable_gym.robots.mia_hand import MiaHandPosition

TEST_POS = np.array([0, 0, 1])
TEST_ORN = pb.getQuaternionFromEuler(np.array([0, 0, 0]))


@pytest.fixture
def robot(simulation):
    robot = MiaHandPosition(
        pb_client=simulation.pb_client,
        world_pos=TEST_POS,
        world_orn=TEST_ORN,
        base_commands=True,
    )
    robot.set_thumb_opp(thumb_adducted=True)

    return robot


def test_mia_hand_position_initial_pose(simulation, robot):

    actual_pose = np.concatenate(robot.multibody_pose.get_pose())
    expected_pose = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    assert_almost_equal(actual_pose, expected_pose)


def test_mia_hand_position_creation(simulation, robot, mia_motors, mia_sensors):
    # check motor creation
    assert set(robot.motors.keys()) == set(mia_motors)
    # check sensor creation
    assert set(robot.sensors.keys()) == set(mia_sensors)


@pytest.mark.skip("TODO")
def test_mia_hand_position_base_movement(simulation, robot):

    robot.perform_command(
        np.array([0.3, -0.2, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    )

    for _ in range(50):
        simulation.step_to_trigger("time_step")

    actual_pose = np.concatenate(robot.multibody_pose.get_pose())
    expected_pose = np.array([0.3, -0.2, 1.1, 0.0, 0.0, 0.0, 1.0])
    assert_almost_equal(actual_pose, expected_pose)
