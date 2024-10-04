import mujoco
import numpy as np
import pytest
from numpy.testing import assert_array_almost_equal, assert_array_equal

import deformable_gym.helpers.asset_manager as am
from deformable_gym.helpers.mj_utils import Pose
from deformable_gym.robots.mj_robot import ShadowHand

JOINTS = [
    "ee_X",
    "ee_Y",
    "ee_Z",
    "ee_OX",
    "ee_OY",
    "ee_OZ",
    "rh_WRJ2",
    "rh_WRJ1",
    "rh_FFJ4",
    "rh_FFJ3",
    "rh_FFJ2",
    "rh_FFJ1",
    "rh_MFJ4",
    "rh_MFJ3",
    "rh_MFJ2",
    "rh_MFJ1",
    "rh_RFJ4",
    "rh_RFJ3",
    "rh_RFJ2",
    "rh_RFJ1",
    "rh_LFJ5",
    "rh_LFJ4",
    "rh_LFJ3",
    "rh_LFJ2",
    "rh_LFJ1",
    "rh_THJ5",
    "rh_THJ4",
    "rh_THJ3",
    "rh_THJ2",
    "rh_THJ1",
]

ACTUATORS_JOINT = [
    "rh_A_WRJ2",
    "rh_A_WRJ1",
    "rh_A_THJ5",
    "rh_A_THJ4",
    "rh_A_THJ3",
    "rh_A_THJ2",
    "rh_A_THJ1",
    "rh_A_FFJ4",
    "rh_A_FFJ3",
    "rh_A_FFJ0",
    "rh_A_MFJ4",
    "rh_A_MFJ3",
    "rh_A_MFJ0",
    "rh_A_RFJ4",
    "rh_A_RFJ3",
    "rh_A_RFJ0",
    "rh_A_LFJ5",
    "rh_A_LFJ4",
    "rh_A_LFJ3",
    "rh_A_LFJ0",
    "ee_A_X",
    "ee_A_Y",
    "ee_A_Z",
    "ee_A_OX",
    "ee_A_OY",
    "ee_A_OZ",
]

ACTUATORS_MOCAP = [
    "rh_A_WRJ2",
    "rh_A_WRJ1",
    "rh_A_THJ5",
    "rh_A_THJ4",
    "rh_A_THJ3",
    "rh_A_THJ2",
    "rh_A_THJ1",
    "rh_A_FFJ4",
    "rh_A_FFJ3",
    "rh_A_FFJ0",
    "rh_A_MFJ4",
    "rh_A_MFJ3",
    "rh_A_MFJ0",
    "rh_A_RFJ4",
    "rh_A_RFJ3",
    "rh_A_RFJ0",
    "rh_A_LFJ5",
    "rh_A_LFJ4",
    "rh_A_LFJ3",
    "rh_A_LFJ0",
]
CTRL_RANGE_JOINT = np.array(
    [
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.005, 0.005],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.005, 0.005],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.005, 0.005],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.005, 0.005],
        [-0.005, 0.005],
        [-0.005, 0.005],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
    ]
)

CTRL_RANGE_MOCAP = np.array(
    [
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.005, 0.005],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.005, 0.005],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.005, 0.005],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
        [-0.01745329, 0.01745329],
    ]
)


@pytest.fixture
def model():
    return am.load_asset("shadow_hand")


@pytest.fixture
def data(model):
    return mujoco.MjData(model)


@pytest.fixture
def shadow_hand_joint():
    return ShadowHand(control_type="joint")


@pytest.fixture
def shadow_hand_mocap():
    return ShadowHand(control_type="mocap")


def test_name(shadow_hand_joint, shadow_hand_mocap):
    assert shadow_hand_joint.name == "shadow_hand"
    assert shadow_hand_mocap.name == "shadow_hand"


def test_n_qpos(shadow_hand_joint, shadow_hand_mocap):
    assert shadow_hand_joint.n_qpos == 30
    assert shadow_hand_mocap.n_qpos == 30


def test_dof(shadow_hand_joint, shadow_hand_mocap):
    assert shadow_hand_joint.dof == 30
    assert shadow_hand_mocap.dof == 30


def test_n_actuator(shadow_hand_joint, shadow_hand_mocap):
    assert shadow_hand_joint.n_actuator == 26
    assert shadow_hand_mocap.n_actuator == 20


def test_ctrl_range(shadow_hand_joint, shadow_hand_mocap):
    assert_array_almost_equal(
        shadow_hand_joint.ctrl_range, CTRL_RANGE_JOINT, decimal=8
    )
    assert_array_almost_equal(
        shadow_hand_mocap.ctrl_range, CTRL_RANGE_MOCAP, decimal=8
    )


def test_joints(shadow_hand_joint, shadow_hand_mocap):
    assert shadow_hand_joint.joints == JOINTS
    assert shadow_hand_mocap.joints == JOINTS


def test_actuators(shadow_hand_joint, shadow_hand_mocap):
    assert shadow_hand_joint.actuators == ACTUATORS_JOINT
    assert shadow_hand_mocap.actuators == ACTUATORS_MOCAP


@pytest.mark.parametrize(
    "pose",
    [
        Pose([0, 0, 0], [np.pi / 2, 0, 0]),
        Pose([1, 1, 1], [np.pi, np.pi, np.pi]),
    ],
)
def test_set_pose(model, data, shadow_hand_joint, pose):
    shadow_hand_joint.set_pose(model, data, pose)
    assert_array_equal(pose.position, model.body(shadow_hand_joint.name).pos)
    assert_array_equal(
        pose.orientation, model.body(shadow_hand_joint.name).quat
    )


def test_get_qpos(model, data, shadow_hand_joint):
    for _ in range(100):
        data.ctrl = 1
        mujoco.mj_step(model, data)
    qpos = shadow_hand_joint.get_qpos(model, data)
    assert_array_almost_equal(qpos, data.qpos)


def test_get_qvel(model, data, shadow_hand_joint):
    for _ in range(100):
        data.ctrl = 1
        mujoco.mj_step(model, data)
    qvel = shadow_hand_joint.get_qvel(model, data)
    assert_array_almost_equal(qvel, data.qvel)
