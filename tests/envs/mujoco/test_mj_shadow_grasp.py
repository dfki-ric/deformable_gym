import numpy as np
import pytest
from gymnasium.spaces import Box
from numpy.testing import (
    assert_allclose,
    assert_array_almost_equal,
    assert_array_equal,
)

from deformable_gym.envs.mujoco.grasp_env import GraspEnv

SEED = 42
# fmt: off
OBS_SPACE_WITHOUT_OBJ_POS = Box(
    low=np.array(
        [
            -0.5     , -0.5     , -0.5     , -1.570796, -1.570796, -1.570796,
            -0.523599, -0.698132, -0.349066, -0.261799,  0.      ,  0.      ,
            -0.349066, -0.261799, 0.      ,  0.      , -0.349066, -0.261799,
            0.      ,  0.      ,  0.      , -0.349066, -0.261799,  0.      ,
            0.      , -1.0472  ,  0.      , -0.20944 , -0.698132, -0.261799
        ]
    ),
    high=np.array(
        [
            0.5     , 0.5     , 0.5     , 1.570796, 1.570796, 1.570796,
            0.174533, 0.488692, 0.349066, 1.5708  , 1.5708  , 1.5708  ,
            0.349066, 1.5708  , 1.5708  , 1.5708  , 0.349066, 1.5708  ,
            1.5708  , 1.5708  , 0.785398, 0.349066, 1.5708  , 1.5708  ,
            1.5708  , 1.0472  , 1.22173 , 0.20944 , 0.698132, 1.5708
        ]
    ),
    shape=(30,),
    dtype=np.float64,
)

OBS_SPACE_WITH_OBJ_POS = Box(
    low=np.array(
        [
            -0.5     , -0.5     , -0.5     , -1.570796, -1.570796, -1.570796,
            -0.523599, -0.698132, -0.349066, -0.261799,  0.      ,  0.      ,
            -0.349066, -0.261799,  0.      ,  0.      , -0.349066, -0.261799,
            0.      ,  0.      ,  0.      , -0.349066, -0.261799,  0.      ,
            0.      , -1.0472  ,  0.      , -0.20944 , -0.698132, -0.261799,
            -np.inf, -np.inf, -np.inf
        ]
    ),
    high=np.array(
        [
            0.5     , 0.5     , 0.5     , 1.570796, 1.570796, 1.570796,
            0.174533, 0.488692, 0.349066, 1.5708  , 1.5708  , 1.5708  ,
            0.349066, 1.5708  , 1.5708  , 1.5708  , 0.349066, 1.5708  ,
            1.5708  , 1.5708  , 0.785398, 0.349066, 1.5708  , 1.5708  ,
            1.5708  , 1.0472  , 1.22173 , 0.20944 , 0.698132, 1.5708  ,
            np.inf, np.inf, np.inf
        ]
    ),
    shape=(33,),
    dtype=np.float64,
)

ACTION_SPACE_JOINT = Box(
    low=np.array(
        [
            -0.01745329, -0.01745329, -0.01745329, -0.01745329, -0.01745329,
            -0.01745329, -0.01745329, -0.01745329, -0.01745329, -0.005     ,
            -0.01745329, -0.01745329, -0.005     , -0.01745329, -0.01745329,
            -0.005     , -0.01745329, -0.01745329, -0.01745329, -0.01745329,
            -0.005     , -0.005     , -0.005     , -0.01745329, -0.01745329,
            -0.01745329
        ]
    ),
    high=np.array(
        [
            0.01745329, 0.01745329, 0.01745329, 0.01745329, 0.01745329,
            0.01745329, 0.01745329, 0.01745329, 0.01745329, 0.005     ,
            0.01745329, 0.01745329, 0.005     , 0.01745329, 0.01745329,
            0.005     , 0.01745329, 0.01745329, 0.01745329, 0.01745329,
            0.005     , 0.005     , 0.005     , 0.01745329, 0.01745329,
            0.01745329
        ]
    ),
    shape=(26,),
    dtype=np.float64,
)

ACTION_SPACE_MOCAP = Box(
    low=np.array(
        [
            -0.005     , -0.005     , -0.005     , -0.03141593, -0.03141593,
            -0.03141593, -0.01745329, -0.01745329, -0.01745329, -0.01745329,
            -0.01745329, -0.01745329, -0.01745329, -0.01745329, -0.01745329,
            -0.005     , -0.01745329, -0.01745329, -0.005     , -0.01745329,
            -0.01745329, -0.005     , -0.01745329, -0.01745329, -0.01745329,
            -0.01745329
        ]
        ),
    high=np.array(
        [
            0.005     , 0.005     , 0.005     , 0.03141593, 0.03141593,
            0.03141593, 0.01745329, 0.01745329, 0.01745329, 0.01745329,
            0.01745329, 0.01745329, 0.01745329, 0.01745329, 0.01745329,
            0.005     , 0.01745329, 0.01745329, 0.005     , 0.01745329,
            0.01745329, 0.005     , 0.01745329, 0.01745329, 0.01745329,
            0.01745329
        ]
        ),
    shape=(26,),
    dtype=np.float64,
)
# fmt: on


def make_env(ctrl_type: str, observable_object_pos: bool):
    return GraspEnv(
        robot_name="shadow_hand",
        obj_name="insole_fixed",
        control_type=ctrl_type,
        observable_object_pos=observable_object_pos,
    )


@pytest.fixture
def env_without_obj_pos():
    env = make_env("joint", False)
    env.action_space.seed(SEED)
    return env


@pytest.fixture
def env_with_obj_pos():
    env = make_env("joint", True)
    env.action_space.seed(SEED)
    return env


@pytest.fixture
def env_joint_ctrl():
    env = make_env("joint", False)
    env.action_space.seed(SEED)
    return env


@pytest.fixture
def env_mocap_ctrl():
    env = make_env("mocap", False)
    env.action_space.seed(SEED)
    return env


def test_observation_space(env_without_obj_pos, env_with_obj_pos):
    assert_array_equal(
        env_without_obj_pos.observation_space.low, OBS_SPACE_WITHOUT_OBJ_POS.low
    )
    assert_array_equal(
        env_without_obj_pos.observation_space.high,
        OBS_SPACE_WITHOUT_OBJ_POS.high,
    )
    assert (
        env_without_obj_pos.observation_space.shape[0]
        == OBS_SPACE_WITHOUT_OBJ_POS.shape[0]
    )
    assert_array_equal(
        env_with_obj_pos.observation_space.low, OBS_SPACE_WITH_OBJ_POS.low
    )
    assert_array_equal(
        env_with_obj_pos.observation_space.high, OBS_SPACE_WITH_OBJ_POS.high
    )
    assert (
        env_with_obj_pos.observation_space.shape[0]
        == OBS_SPACE_WITH_OBJ_POS.shape[0]
    )


def test_action_space(env_joint_ctrl, env_mocap_ctrl):
    assert_array_almost_equal(
        env_joint_ctrl.action_space.low, ACTION_SPACE_JOINT.low, decimal=8
    )
    assert_array_almost_equal(
        env_joint_ctrl.action_space.high, ACTION_SPACE_JOINT.high, decimal=8
    )
    assert env_joint_ctrl.action_space.shape[0] == ACTION_SPACE_JOINT.shape[0]
    assert_array_almost_equal(
        env_mocap_ctrl.action_space.low, ACTION_SPACE_MOCAP.low, decimal=8
    )
    assert_array_almost_equal(
        env_mocap_ctrl.action_space.high, ACTION_SPACE_MOCAP.high, decimal=8
    )
    assert env_mocap_ctrl.action_space.shape[0] == ACTION_SPACE_MOCAP.shape[0]


def test_reset(env_with_obj_pos, env_without_obj_pos):
    obs, info = env_without_obj_pos.reset(seed=SEED)
    assert obs.shape[0] == 30
    env_without_obj_pos.close()
    obs, info = env_with_obj_pos.reset(seed=SEED)
    assert obs.shape[0] == 33
    env_with_obj_pos.close()


def test_episode_reproducibility(env_without_obj_pos: GraspEnv):
    observations = []
    termination_flags = []
    actions = []
    # env_without_obj_pos.render_mode = "rgb_array"
    for _ in range(2):
        observation, _ = env_without_obj_pos.reset(seed=SEED)
        env_without_obj_pos.action_space.seed(SEED)
        observations.append([observation])
        terminated = False
        termination_flags.append([terminated])
        actions.append([])
        while not terminated:
            action = env_without_obj_pos.action_space.sample()
            actions[-1].append(action)
            observation, reward, terminated, truncated, info = (
                env_without_obj_pos.step(action)
            )
            observations[-1].append(observation)
            termination_flags[-1].append(terminated)
    env_without_obj_pos.close()
    assert_allclose(actions[0], actions[1])
    assert_allclose(observations[0], observations[1])
    assert_allclose(termination_flags[0], termination_flags[1])
