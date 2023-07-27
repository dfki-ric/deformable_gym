import numpy as np
import pybullet as pb
import pytest
from deformable_gym.envs.pybullet_tools import BulletSimulation
from deformable_gym.objects.bullet_object import ObjectFactory, UrdfObject
from numpy.testing import assert_array_almost_equal


TEST_POS = np.array([0, 0, 1])
TEST_ORN = np.array([0, 0, 0])


def test_sphere(simulation):
    obj, _, _ = ObjectFactory().create("sphere", object_position=TEST_POS, object_orientation=TEST_ORN)
    pose = obj.get_pose()
    assert_array_almost_equal(pose, np.array([0, 0, 1, 1, 0, 0, 0]))

    vertices = obj.get_vertices()
    assert len(vertices) == 0

    assert obj.get_id() == 0
