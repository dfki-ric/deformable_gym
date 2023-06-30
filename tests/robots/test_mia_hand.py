import numpy as np
import pybullet as pb
from deformable_gym.envs.pybullet_tools import BulletSimulation
from deformable_gym.robots.mia_hand import MiaHandVelocity
from numpy.testing import assert_almost_equal

def test_mia_hand_velocity():
    simulation = BulletSimulation(time_delta=0.0001, mode=pb.DIRECT)
    simulation.timing.add_subsystem("time_step", 100, None)

    robot = MiaHandVelocity(verbose=1)
    robot.set_thumb_opp(thumb_adducted=True)
    simulation.add_robot(robot)

    simulation.camera.reset(
        position=robot.get_ee_pose()[:3], pitch=0, yaw=110, distance=1)

    robot.perform_command(np.array([1.0, 1.0, 1.0]))
    for _ in range(50):
        simulation.step_to_trigger("time_step")
    joint_positions = robot.get_joint_positions()
    expected_angles = [0.5, 0.5, 0.5, 0.5, 0.0, 0.5]
    assert_almost_equal(joint_positions, expected_angles)

    robot.perform_command(np.array([0.0, 0.0, 0.0]))
    for _ in range(50):
        simulation.step_to_trigger("time_step")
    joint_positions = robot.get_joint_positions()
    expected_angles = [0.5, 0.5, 0.5, 0.5, 0.0, 0.5]
    assert_almost_equal(joint_positions, expected_angles)
