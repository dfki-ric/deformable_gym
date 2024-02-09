import pytest
from gymnasium.wrappers import RescaleAction
from numpy.testing import assert_allclose

from deformable_gym.envs.floating_mia_grasp_env import FloatingMiaGraspEnv


@pytest.fixture
def env():
    return FloatingMiaGraspEnv(
        gui=False,
        verbose=True,
        horizon=10,
        object_name="insole_on_conveyor_belt/back")


action_space_dims_expected = 10
SEED = 42


def test_action_space_dims(env: FloatingMiaGraspEnv):
    action_space = env.action_space
    assert action_space.shape[0] == action_space_dims_expected


def test_obs_space_dims(env: FloatingMiaGraspEnv):
    if env._observable_object_pos:
        obs_space_dims_expected = 19
    else:
        obs_space_dims_expected = 16

    obs_space = env.observation_space
    assert obs_space.shape[0] == obs_space_dims_expected


def test_initial_obs(env: FloatingMiaGraspEnv):
    obs, info = env.reset(seed=SEED)
    if env._observable_object_pos:
        obs_space_dims_expected = 19
    else:
        obs_space_dims_expected = 16
    assert len(obs) == obs_space_dims_expected


def test_initial_sensor_info(env: FloatingMiaGraspEnv):
    sensor_readings = []
    env.action_space.seed(SEED)

    for _ in range(2):
        observation, _ = env.reset(seed=SEED)
        sensor_readings.append(observation)

        n_steps = 5
        for _ in range(n_steps):
            action = env.action_space.sample()
            env.step(action)

    assert_allclose(sensor_readings[0], sensor_readings[1])


def test_episode_reproducibility():
    observations = []
    termination_flags = []

    env = FloatingMiaGraspEnv(verbose=False,
                              horizon=10,
                              gui=False,
                              observable_object_pos=True,
                              object_name="insole_on_conveyor_belt/back",
                              difficulty_mode="hard",
                              )
    env = RescaleAction(env, 0., 1.)

    env.action_space.seed(SEED)

    for _ in range(2):
        observation, _ = env.reset(seed=SEED)
        observations.append(observation)
        terminated = False
        while not terminated:
            action = env.action_space.sample()
            observation, reward, terminated, truncated, info = env.step(action)

            observations.append(observation)
            termination_flags.append(terminated)

    assert_allclose(observations[0], observations[1])
    assert_allclose(termination_flags[0], termination_flags[1])


def test_eps_done(env: FloatingMiaGraspEnv):
    if env._observable_object_pos:
        obs_space_dims_expected = 19
    else:
        obs_space_dims_expected = 16

    env.action_space.seed(SEED)

    for t in range(9):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)

        assert len(obs) == obs_space_dims_expected
        assert isinstance(reward, float)
        assert isinstance(terminated, bool)
        assert not terminated

    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)

    assert terminated
