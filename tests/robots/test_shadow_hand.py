import numpy as np
import os
import pybullet as pb
from pathlib import Path
from deformable_gym.envs.bullet_simulation import BulletSimulation
from deformable_gym.robots.base_robot import BaseRobot
from numpy.testing import assert_almost_equal

URDF_PATH = os.path.join(Path(os.path.dirname(__file__)).parent.parent.absolute(), "robots/urdf/shadow_hand.urdf")

SHADOW_MOTORS = ("rh_WRJ2", "rh_WRJ1", "rh_THJ5", "rh_THJ4", "rh_THJ3", "rh_THJ2", "rh_THJ1", "rh_LFJ5",
                 "rh_LFJ4", "rh_LFJ3", "rh_LFJ2", "rh_RFJ1", "rh_RFJ4", "rh_RFJ3", "rh_RFJ2", "rh_RFJ1",
                 "rh_MFJ4", "rh_MFJ3", "rh_MFJ2", "rh_MFJ1", "rh_FFJ4", "rh_FFJ3", "rh_FFJ2", "rh_FFJ1")
# SHADOW_SENSORS = ("j_index_sensor", "j_middle_sensor", "j_thumb_sensor")


def test_shadow_hand():
    simulation = BulletSimulation(time_delta=.001, mode=pb.DIRECT)

    robot = BaseRobot(urdf_path=URDF_PATH)

    found_motors = robot.motors.keys()

    # check motor creation
    for motor in SHADOW_MOTORS:
        assert motor in found_motors

    # check initial pose
    assert_almost_equal(robot.initial_position, np.zeros(3))
    assert_almost_equal(robot.initial_orientation, pb.getQuaternionFromEuler(np.zeros(3)))

    simulation.disconnect()



