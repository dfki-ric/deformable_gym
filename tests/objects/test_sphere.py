import numpy as np
import pybullet as pb
import pytest
from numpy.testing import assert_array_almost_equal

from deformable_gym.envs.bullet_simulation import BulletSimulation
from deformable_gym.objects.bullet_object import ObjectFactory, UrdfObject

TEST_POS = np.array([0, 0, 1])
TEST_ORN = np.array([0, 0, 0])


def test_sphere(simulation):
    obj, _, _ = ObjectFactory(simulation.pb_client).create(
        "sphere", object_position=TEST_POS, object_orientation=TEST_ORN
    )
    pose = obj.get_pose()
    assert_array_almost_equal(pose, np.array([0, 0, 1, 1, 0, 0, 0]))

    vertices = obj.get_vertices()
    assert len(vertices) == 0

    assert obj.get_id() == 0
