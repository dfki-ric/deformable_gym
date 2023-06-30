import numpy as np
import pybullet as pb
from deformable_gym.envs.pybullet_tools import BulletSimulation
from deformable_gym.objects.bullet_object import (
    SphereObject, MeshObject, UrdfObject, SoftObject, Insole, PillowSmall)
from numpy.testing import assert_array_almost_equal


def test_sphere():
    simulation = BulletSimulation(
        time_delta=0.01, mode=pb.DIRECT, gravity=-9.81)
    simulation.timing.add_subsystem("time_step", 1, None)

    try:
        obj = SphereObject(client_id=simulation.get_physics_client_id())
        pose = obj.get_pose()
        assert_array_almost_equal(pose, np.array([0, 0, 0, 1, 0, 0, 0]))

        vertices = obj.get_vertices()
        assert len(vertices) == 0

        obj.remove_anchors()
        simulation.step_to_trigger("time_step")

        pose = obj.get_pose()
        assert_array_almost_equal(pose, np.array([0, 0, -4.606547, 1, 0, 0, 0]))

        obj.reset()
        pose = obj.get_pose()
        assert_array_almost_equal(pose, np.array([0, 0, 0, 1, 0, 0, 0]))

        assert obj.get_id() == 0
    finally:
        simulation.disconnect()

def test_urdf():
    simulation = BulletSimulation(
        time_delta=0.01, mode=pb.DIRECT, gravity=-9.81)
    simulation.timing.add_subsystem("time_step", 1, None)

    try:
        obj = UrdfObject("plane.urdf", world_pos=(0, 0, 0), fixed=True,
                         client_id=simulation.get_physics_client_id())
        pose = obj.get_pose()
        assert_array_almost_equal(pose, np.array([0, 0, 0, 1, 0, 0, 0]))

        vertices = obj.get_vertices()
        assert len(vertices) == 0

        simulation.step_to_trigger("time_step")

        pose = obj.get_pose()
        assert_array_almost_equal(pose, np.array([0, 0, 0, 1, 0, 0, 0]))

        obj.reset()
        pose = obj.get_pose()
        assert_array_almost_equal(pose, np.array([0, 0, 0, 1, 0, 0, 0]))

        assert obj.get_id() == 0
    finally:
        simulation.disconnect()

def test_insole():
    simulation = BulletSimulation(
        time_delta=0.01, mode=pb.DIRECT, gravity=-9.81,
        soft=True)
    simulation.timing.add_subsystem("time_step", 1, None)

    try:
        obj = Insole(insole_markers2world=np.eye(4), fixed=False,
                     client_id=simulation.get_physics_client_id())
        pose = obj.get_pose()
        #assert_array_almost_equal(pose, np.array([0, 0, 0, 1, 0, 0, 0]))

        #vertices = obj.get_vertices()
        #assert len(vertices) == 516

        obj.remove_anchors()
        simulation.step_to_trigger("time_step")

        pose = obj.get_pose()
        #assert_array_almost_equal(pose, np.array([-0.52, 0.0, 1.05, 1, 0, 0, 0]))

        obj.reset()
        pose = obj.get_pose()
        #assert_array_almost_equal(pose, np.array([-0.52, 0.0, 1.05, 1, 0, 0, 0]))

        assert obj.get_id() == 0
    finally:
        simulation.disconnect()


def test_pillow():
    simulation = BulletSimulation(
        time_delta=0.01, mode=pb.DIRECT, gravity=-9.81,
        soft=True)
    simulation.timing.add_subsystem("time_step", 1, None)

    try:
        obj = PillowSmall(pillow_markers2world=np.eye(4), fixed=False,
                          client_id=simulation.get_physics_client_id())
        pose = obj.get_pose()
        #assert_array_almost_equal(pose, np.array([0, 0, 0, 1, 0, 0, 0]))

        #vertices = obj.get_vertices()
        #assert len(vertices) == 516

        obj.remove_anchors()
        simulation.step_to_trigger("time_step")

        pose = obj.get_pose()
        #assert_array_almost_equal(pose, np.array([-0.52, 0.0, 1.05, 1, 0, 0, 0]))

        obj.reset()
        pose = obj.get_pose()
        #assert_array_almost_equal(pose, np.array([-0.52, 0.0, 1.05, 1, 0, 0, 0]))

        assert obj.get_id() == 0
    finally:
        simulation.disconnect()

