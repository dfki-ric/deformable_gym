import mujoco
import numpy as np
import pytest
from numpy.testing import assert_array_equal

import deformable_gym.helpers.asset_manager as am
from deformable_gym.helpers import mj_utils as mju
from deformable_gym.objects.mj_object import InsoleFixed


@pytest.fixture
def obj():
    return InsoleFixed()


@pytest.fixture
def model():
    return am.load_asset("insole_fixed")


@pytest.fixture
def data(model):
    return mujoco.MjData(model)


def test_name(obj):
    assert obj.name == "insole_fixed"


def test_eq_constraints(obj):
    assert obj.eq_constraints == [
        "fix_1",
        "fix_2",
        "fix_3",
        "fix_4",
        "fix_5",
        "fix_6",
        "fix_7",
        "fix_8",
    ]


def test_eq_constraints_to_disable(obj):
    assert obj.eq_constraints_to_disable == [
        "fix_1",
        "fix_2",
        "fix_3",
        "fix_4",
        "fix_5",
        "fix_6",
        "fix_7",
        "fix_8",
    ]


@pytest.mark.parametrize(
    "pose",
    [
        mju.Pose([0, 0, 0], [np.pi / 2, 0, 0]),
        mju.Pose([1, 1, 1], [np.pi, np.pi, np.pi]),
    ],
)
def test_set_pose(model, data, obj, pose):
    obj.set_pose(model, data, pose)
    assert_array_equal(pose.position, model.body("insole_fixed").pos)
    assert_array_equal(pose.orientation, model.body("insole_fixed").quat)
