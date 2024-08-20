import numpy as np
import pytest
from gymnasium.spaces import Box
from numpy.testing import assert_allclose, assert_array_equal

from deformable_gym.envs.mujoco.shadow_grasp import ShadowHandGrasp

SEED = 42
OBS_SPACE = Box(
    low=np.array(
        [
            -0.5,
            -0.5,
            -0.5,
            -1.570796,
            -1.570796,
            -1.570796,
            -0.523599,
            -0.698132,
            -0.349066,
            -0.261799,
            0.0,
            0.0,
            -0.349066,
            -0.261799,
            0.0,
            0.0,
            -0.349066,
            -0.261799,
            0.0,
            0.0,
            0.0,
            -0.349066,
            -0.261799,
            0.0,
            0.0,
            -1.0472,
            0.0,
            -0.20944,
            -0.698132,
            -0.261799,
        ]
    ),
    high=np.array(
        [
            0.5,
            0.5,
            0.5,
            1.570796,
            1.570796,
            1.570796,
            0.174533,
            0.488692,
            0.349066,
            1.5708,
            1.5708,
            1.5708,
            0.349066,
            1.5708,
            1.5708,
            1.5708,
            0.349066,
            1.5708,
            1.5708,
            1.5708,
            0.785398,
            0.349066,
            1.5708,
            1.5708,
            1.5708,
            1.0472,
            1.22173,
            0.20944,
            0.698132,
            1.5708,
        ]
    ),
    shape=(30,),
    dtype=np.float64,
)
OBS_SPACE_W_OBJ_POS = Box(
    low=np.array(
        [
            -0.5,
            -0.5,
            -0.5,
            -1.570796,
            -1.570796,
            -1.570796,
            -0.523599,
            -0.698132,
            -0.349066,
            -0.261799,
            0.0,
            0.0,
            -0.349066,
            -0.261799,
            0.0,
            0.0,
            -0.349066,
            -0.261799,
            0.0,
            0.0,
            0.0,
            -0.349066,
            -0.261799,
            0.0,
            0.0,
            -1.0472,
            0.0,
            -0.20944,
            -0.698132,
            -0.261799,
            -np.inf,
            -np.inf,
            -np.inf,
        ]
    ),
    high=np.array(
        [
            0.5,
            0.5,
            0.5,
            1.570796,
            1.570796,
            1.570796,
            0.174533,
            0.488692,
            0.349066,
            1.5708,
            1.5708,
            1.5708,
            0.349066,
            1.5708,
            1.5708,
            1.5708,
            0.349066,
            1.5708,
            1.5708,
            1.5708,
            0.785398,
            0.349066,
            1.5708,
            1.5708,
            1.5708,
            1.0472,
            1.22173,
            0.20944,
            0.698132,
            1.5708,
            np.inf,
            np.inf,
            np.inf,
        ]
    ),
    shape=(33,),
    dtype=np.float64,
)
ACTION_SPACE = Box(
    low=np.array(
        [
            -0.523599,
            -0.698132,
            -1.0472,
            0.0,
            -0.20944,
            -0.698132,
            -0.261799,
            -0.349066,
            -0.261799,
            0.0,
            -0.349066,
            -0.261799,
            0.0,
            -0.349066,
            -0.261799,
            0.0,
            0.0,
            -0.349066,
            -0.261799,
            0.0,
            -1.0,
            -1.0,
            -1.0,
            -1.0,
            -1.0,
            -1.0,
        ]
    ),
    high=np.array(
        [
            0.174533,
            0.488692,
            1.0472,
            1.22173,
            0.20944,
            0.698132,
            1.5708,
            0.349066,
            1.5708,
            3.1415,
            0.349066,
            1.5708,
            3.1415,
            0.349066,
            1.5708,
            3.1415,
            0.785398,
            0.349066,
            1.5708,
            3.1415,
            1.0,
            1.0,
            1.0,
            1.0,
            1.0,
            1.0,
        ]
    ),
    shape=(26,),
    dtype=np.float64,
)


@pytest.fixture
def env():
    env = ShadowHandGrasp(
        obj_name="insole_fixed", gui=False, observable_object_pos=False
    )
    env.action_space.seed(SEED)
    return env


@pytest.fixture
def env_w_obj_pos():
    env = ShadowHandGrasp(
        obj_name="insole_fixed", gui=False, observable_object_pos=True
    )
    env.action_space.seed(SEED)
    return env


def test_observation_space(
    env: ShadowHandGrasp, env_w_obj_pos: ShadowHandGrasp
):
    assert_array_equal(env.observation_space.low, OBS_SPACE.low)
    assert_array_equal(env.observation_space.high, OBS_SPACE.high)
    assert env.observation_space.shape[0] == OBS_SPACE.shape[0]
    assert_array_equal(
        env_w_obj_pos.observation_space.low, OBS_SPACE_W_OBJ_POS.low
    )
    assert (
        env_w_obj_pos.observation_space.shape[0] == OBS_SPACE_W_OBJ_POS.shape[0]
    )


def test_action_space(env: ShadowHandGrasp):
    assert_array_equal(env.action_space.low, ACTION_SPACE.low)
    assert_array_equal(env.action_space.high, ACTION_SPACE.high)
    assert env.action_space.shape[0] == ACTION_SPACE.shape[0]


def test_reset(env: ShadowHandGrasp, env_w_obj_pos: ShadowHandGrasp):
    obs, info = env.reset(seed=SEED)
    assert obs.shape[0] == 30
    obs, info = env_w_obj_pos.reset(seed=SEED)
    assert obs.shape[0] == 33


def test_episode_reproducibility(env):
    observations = []
    termination_flags = []
    actions = []

    for _ in range(2):
        observation, _ = env.reset(seed=SEED)
        env.action_space.seed(SEED)

        observations.append([observation])
        terminated = False
        termination_flags.append([terminated])
        actions.append([])
        while not terminated:
            action = env.action_space.sample()
            actions[-1].append(action)
            observation, reward, terminated, truncated, info = env.step(action)

            observations[-1].append(observation)
            termination_flags[-1].append(terminated)

    assert_allclose(actions[0], actions[1])
    assert_allclose(observations[0], observations[1])
    assert_allclose(termination_flags[0], termination_flags[1])
