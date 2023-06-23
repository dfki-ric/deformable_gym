import numpy as np
import pybullet as pb
from deformable_gym.envs.pybullet_tools import BulletSimulation
from deformable_gym.robots.mia_hand import MiaHandPosition
from numpy.testing import assert_array_almost_equal


def test_via_point_control():

    simulation = BulletSimulation(
        mode=pb.DIRECT,
        gravity=-9.81,
        soft=True)

    try:
        robot = MiaHandPosition(verbose=False,
                                world_pos=np.array([0, 0, 1]),
                                world_orn=np.array([0, 0, 0]),
                                debug_visualization=False,
                                base_commands=True)

        simulation.add_robot(robot)

        # test move vertically
        target_pose = np.array([0, 0, 2, 0, 0, 0, 1])
        robot.move_base_to(target_pose, np.array([0, 0, 0.001, 0, 0, 0, 0]))
        reached_pose = np.hstack((robot.multibody_pose.get_pose()))
        assert_array_almost_equal(target_pose, reached_pose, decimal=4)

        # test move horizontally
        target_pose = np.array([0, 1, 2, 0, 0, 0, 1])
        robot.move_base_to(target_pose, np.array([0.001, 0.001, 0.001, 0, 0, 0, 0]))
        reached_pose = np.hstack((robot.multibody_pose.get_pose()))
        assert_array_almost_equal(target_pose, reached_pose, decimal=4)

        # test move diagonally
        target_pose = np.array([1, 0, 4, 0, 0, 0, 1])
        robot.move_base_to(target_pose, np.array([0.001, 0.001, 0.001, 0, 0, 0, 0]))
        reached_pose = np.hstack((robot.multibody_pose.get_pose()))
        assert_array_almost_equal(target_pose, reached_pose, decimal=4)

    finally:
        simulation.disconnect()
