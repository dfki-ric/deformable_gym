import numpy as np
import pybullet as pb
import pytest
from deformable_gym.envs.pybullet_tools import BulletSimulation
from deformable_gym.objects.bullet_object import ObjectFactory, UrdfObject
from numpy.testing import assert_array_almost_equal


TEST_POS = np.array([0, 0, 1])
TEST_ORN = np.array([0, 0, 0])


@pytest.fixture
def simulation():
    sim = BulletSimulation(mode=pb.DIRECT, verbose_dt=10000, soft=True)
    sim.timing.add_subsystem("time_step", 100, None)
    return sim


def test_sphere(simulation):
    obj, _, _ = ObjectFactory().create("sphere", object_position=TEST_POS, object_orientation=TEST_ORN)
    pose = obj.get_pose()
    assert_array_almost_equal(pose, np.array([0, 0, 1, 1, 0, 0, 0]))

    vertices = obj.get_vertices()
    assert len(vertices) == 0

    assert obj.get_id() == 0


def test_urdf(simulation):
    obj = UrdfObject("plane.urdf", world_pos=TEST_POS, world_orn=TEST_ORN, fixed=True,
                     client_id=simulation.get_physics_client_id())
    pose = obj.get_pose()
    assert_array_almost_equal(pose, np.array([0, 0, 1, 1, 0, 0, 0]))

    vertices = obj.get_vertices()
    assert len(vertices) == 0

    assert obj.get_id() == 0


def test_insole(simulation):
    obj, _, _ = ObjectFactory().create("insole", object_position=TEST_POS, object_orientation=TEST_ORN)
    pose = obj.get_pose()
    assert_array_almost_equal(pose, np.array([0, 0, 1, 1, 0, 0, 0]), decimal=1)

    vertices = obj.get_vertices()
    assert len(vertices) == 516

    assert obj.get_id() == 0


def test_pillow(simulation):
    obj, _, _ = ObjectFactory().create("pillow_small", object_position=TEST_POS, object_orientation=TEST_ORN)
    pose = obj.get_pose()
    assert_array_almost_equal(pose, np.array([0, 0, 1, 1, 0, 0, 0]), decimal=1)

    vertices = obj.get_vertices()
    assert len(vertices) == 738

    assert obj.get_id() == 0
