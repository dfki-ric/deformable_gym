import pytest
from deformable_gym.envs.floating_shadow_grasp_env import FloatingShadowGraspEnv
from numpy.testing import assert_allclose

SEED = 42


@pytest.fixture
def env():
    return FloatingShadowGraspEnv(
        gui=False,
        verbose=True,
        horizon=10,
        object_name="insole",
        observable_object_pos=True,
    )


@pytest.mark.skip("TODO")
def test_action_space_dims(env):
    action_space = env.action_space
    assert action_space.shape[0] == 10


@pytest.mark.skip("TODO")
def test_obs_space_dims(env):
    obs_space = env.observation_space
    assert obs_space.shape[0] == 28


@pytest.mark.skip("TODO")
def test_initial_obs(env):
    obs, info = env.reset()
    assert len(obs) == 18


def test_ep_termination(env):
    env.reset()
    for t in range(9):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)

        assert len(obs) == 18
        assert isinstance(reward, float)
        assert isinstance(terminated, bool)
        assert not terminated

    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)

    assert terminated


def test_initial_sensor_info(env: FloatingShadowGraspEnv):
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
