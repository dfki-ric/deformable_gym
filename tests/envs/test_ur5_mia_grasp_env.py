import pytest
from deformable_gym.envs.ur5_mia_grasp_env import UR5MiaGraspEnv


@pytest.fixture
def env():
    return UR5MiaGraspEnv(
        gui=False,
        verbose=True,
        horizon=10,
        object_name="insole",
        #observable_object_pos=True,
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


@pytest.mark.skip("TODO")
def test_eps_done(env):
    env.reset()
    for t in range(9):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)

        assert len(obs) == 16
        assert isinstance(reward, float)
        assert isinstance(terminated, bool)
        assert not terminated

    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)

    assert terminated

