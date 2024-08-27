import mujoco
import numpy as np
import pytest
from numpy.testing import assert_array_almost_equal, assert_array_equal

from deformable_gym.helpers.asset_manager import AssetManager
from deformable_gym.helpers.mj_utils import Pose
from deformable_gym.robots.mj_robot import ShadowHand

JOINTS = [
    "ee_X",
    "ee_Y",
    "ee_Z",
    "ee_OX",
    "ee_OY",
    "ee_OZ",
    "lh_WRJ2",
    "lh_WRJ1",
    "lh_FFJ4",
    "lh_FFJ3",
    "lh_FFJ2",
    "lh_FFJ1",
    "lh_MFJ4",
    "lh_MFJ3",
    "lh_MFJ2",
    "lh_MFJ1",
    "lh_RFJ4",
    "lh_RFJ3",
    "lh_RFJ2",
    "lh_RFJ1",
    "lh_LFJ5",
    "lh_LFJ4",
    "lh_LFJ3",
    "lh_LFJ2",
    "lh_LFJ1",
    "lh_THJ5",
    "lh_THJ4",
    "lh_THJ3",
    "lh_THJ2",
    "lh_THJ1",
]

ACTUATORS = [
    "lh_A_WRJ2",
    "lh_A_WRJ1",
    "lh_A_THJ5",
    "lh_A_THJ4",
    "lh_A_THJ3",
    "lh_A_THJ2",
    "lh_A_THJ1",
    "lh_A_FFJ4",
    "lh_A_FFJ3",
    "lh_A_FFJ0",
    "lh_A_MFJ4",
    "lh_A_MFJ3",
    "lh_A_MFJ0",
    "lh_A_RFJ4",
    "lh_A_RFJ3",
    "lh_A_RFJ0",
    "lh_A_LFJ5",
    "lh_A_LFJ4",
    "lh_A_LFJ3",
    "lh_A_LFJ0",
    "ee_A_X",
    "ee_A_Y",
    "ee_A_Z",
    "ee_A_OX",
    "ee_A_OY",
    "ee_A_OZ",
]

CTRL_RANGE = np.array(
    [
        [-0.523599, 0.174533],
        [-0.698132, 0.488692],
        [-1.0472, 1.0472],
        [0.0, 1.22173],
        [-0.20944, 0.20944],
        [-0.698132, 0.698132],
        [-0.261799, 1.5708],
        [-0.349066, 0.349066],
        [-0.261799, 1.5708],
        [0.0, 3.1415],
        [-0.349066, 0.349066],
        [-0.261799, 1.5708],
        [0.0, 3.1415],
        [-0.349066, 0.349066],
        [-0.261799, 1.5708],
        [0.0, 3.1415],
        [0.0, 0.785398],
        [-0.349066, 0.349066],
        [-0.261799, 1.5708],
        [0.0, 3.1415],
        [-1.0, 1.0],
        [-1.0, 1.0],
        [-1.0, 1.0],
        [-1.0, 1.0],
        [-1.0, 1.0],
        [-1.0, 1.0],
    ]
)


@pytest.fixture
def model():
    return AssetManager().load_asset("shadow_hand")


@pytest.fixture
def data(model):
    return mujoco.MjData(model)


@pytest.fixture
def shadow_hand():
    return ShadowHand()


def test_name(shadow_hand):
    assert shadow_hand.name == "shadow_hand"


def test_n_qpos(shadow_hand):
    assert shadow_hand.n_qpos == 30


def test_dof(shadow_hand):
    assert shadow_hand.dof == 30


def test_n_actuator(shadow_hand):
    assert shadow_hand.n_actuator == 26


def test_ctrl_range(shadow_hand):
    assert_array_equal(shadow_hand.ctrl_range, CTRL_RANGE)


def test_joints(shadow_hand):
    assert shadow_hand.joints == JOINTS


def test_actuators(shadow_hand):
    assert shadow_hand.actuators == ACTUATORS


@pytest.mark.parametrize(
    "pose",
    [
        Pose([0, 0, 0], [np.pi / 2, 0, 0]),
        Pose([1, 1, 1], [np.pi, np.pi, np.pi]),
    ],
)
def test_set_pose(model, data, shadow_hand, pose):
    shadow_hand.set_pose(model, data, pose)
    assert_array_equal(pose.position, model.body(shadow_hand.name).pos)
    assert_array_equal(pose.orientation, model.body(shadow_hand.name).quat)


def test_get_qpos(model, data, shadow_hand):
    for _ in range(100):
        data.ctrl = 1
        mujoco.mj_step(model, data)
    qpos = shadow_hand.get_qpos(model, data)
    assert_array_almost_equal(qpos, data.qpos)


def test_get_qvel(model, data, shadow_hand):
    for _ in range(100):
        data.ctrl = 1
        mujoco.mj_step(model, data)
    qvel = shadow_hand.get_qvel(model, data)
    assert_array_almost_equal(qvel, data.qvel)
