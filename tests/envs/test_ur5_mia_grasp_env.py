import pytest
from numpy.testing import assert_allclose

from deformable_gym.envs.ur5_mia_grasp_env import UR5MiaGraspEnv


@pytest.fixture
def env():
    return UR5MiaGraspEnv(
        verbose=False,
        horizon=10,
        object_name="insole",
    )


action_space_dims_expected = 10
observation_space_dims_expected = 16
SEED = 42


def test_action_space_dims(env):
    action_space = env.action_space
    assert action_space.shape[0] == action_space_dims_expected


def test_obs_space_dims(env):
    if env._observable_object_pos:
        obs_space_dims_expected = observation_space_dims_expected + 3
    else:
        obs_space_dims_expected = observation_space_dims_expected

    obs_space = env.observation_space
    assert obs_space.shape[0] == obs_space_dims_expected


def test_episode_reproducibility(env: UR5MiaGraspEnv):
    observations = []
    termination_flags = []
    actions = []

    env = UR5MiaGraspEnv(
        verbose=False,
        horizon=3,
        object_name="insole_on_conveyor_belt/back",
    )

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


def test_eps_done(env):
    env.reset()
    for t in range(9):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)

        assert len(obs) == observation_space_dims_expected
        assert isinstance(reward, float)
        assert isinstance(terminated, bool)
        assert not terminated

    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)

    assert terminated
