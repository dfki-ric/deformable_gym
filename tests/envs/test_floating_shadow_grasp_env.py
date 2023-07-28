import pytest
from deformable_gym.envs.floating_shadow_grasp_env import FloatingShadowGraspEnv


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
    obs = env.reset()
    assert len(obs) == 18


def test_eps_done(env):
    env.reset()
    for t in range(9):
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)

        assert len(obs) == 18
        assert isinstance(reward, float)
        assert isinstance(done, bool)
        assert not done

    action = env.action_space.sample()
    obs, reward, done, info = env.step(action)

    assert done

