import numpy as np
import os
import pybullet as pb
from pathlib import Path
from deformable_gym.envs.bullet_simulation import BulletSimulation
from deformable_gym.robots.mia_hand import MiaHandVelocity
from deformable_gym.robots.base_robot import BaseRobot
from numpy.testing import assert_almost_equal

URDF_PATH = os.path.join(Path(os.path.dirname(__file__)).parent.parent.absolute(), "robots/urdf/mia_hand.urdf")

MIA_MOTORS = ("j_thumb_fle", "j_thumb_opp", "j_index_fle",  "j_mrl_fle", "j_ring_fle", "j_little_fle")
MIA_SENSORS = ("j_index_sensor", "j_middle_sensor", "j_thumb_sensor")


def test_mia_hand():
    simulation = BulletSimulation(time_delta=.001, mode=pb.DIRECT)
    simulation.timing.add_subsystem("time_step", 100, None)

    robot = BaseRobot(urdf_path=URDF_PATH)

    found_motors = robot.motors.keys()

    # check motor creation
    for motor in MIA_MOTORS:
        assert motor in found_motors

    found_sensors = robot.sensors.keys()

    # check sensor creation
    for sensor in MIA_SENSORS:
        assert sensor in found_sensors

    # check initial pose
    assert_almost_equal(robot.initial_position, np.zeros(3))
    assert_almost_equal(robot.initial_orientation, pb.getQuaternionFromEuler(np.zeros(3)))

    # check get position and velocity
    assert_almost_equal(robot.get_joint_positions(), np.zeros_like(found_motors))



    print(np.hstack(robot.base_pose.get_pose()))

    simulation.disconnect()


def test_mia_hand_velocity():
    simulation = BulletSimulation(time_delta=.001, mode=pb.DIRECT)
    simulation.timing.add_subsystem("time_step", 100, None)

    robot = MiaHandVelocity(verbose=1)
    robot.set_thumb_opp(thumb_adducted=True)
    simulation.add_robot(robot)

    #simulation.camera.reset(position=robot.get_ee_pose()[:3], pitch=0, yaw=110, distance=1)

    robot.perform_command(np.array([1.0, 1.0, 1.0]))
    for _ in range(100):
        simulation.step_to_trigger("time_step")

    joint_positions = robot.get_joint_positions()
    expected_angles = np.array([0.5, 0.5, 0.5, 0.5, 0.0, 0.5])
    #assert_almost_equal(joint_positions, expected_angles)

    robot.perform_command(np.array([0.0, 0.0, 0.0]))
    for _ in range(100):
        simulation.step_to_trigger("time_step")

    joint_positions = robot.get_joint_positions()
    expected_angles = np.array([0.5, 0.5, 0.5, 0.5, 0.0, 0.5])
    #assert_almost_equal(joint_positions, expected_angles)

    simulation.disconnect()

if __name__ == '__main__':
    test_mia_hand()

