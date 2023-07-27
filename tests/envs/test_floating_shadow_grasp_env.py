import pytest
from deformable_gym.envs.floating_shadow_grasp_env import FloatingShadowGraspEnv


@pytest.fixture
def env():
    return FloatingShadowGraspEnv(
        gui=False,
        verbose=True,
        horizon=100,
        object_name="insole_on_conveyor_belt/back",
        observable_object_pos=True,
        difficulty_mode="hard")


action_space_dims_expected = 10


@pytest.mark.skip("TODO")
def test_action_space_dims(env):
    action_space = env.action_space
    assert action_space.shape[0] == action_space_dims_expected


@pytest.mark.skip("TODO")
def test_obs_space_dims(env):
    if env._observable_object_pos:
        obs_space_dims_expected = 19
    else:
        obs_space_dims_expected = 16

    obs_space = env.observation_space
    assert obs_space.shape[0] == obs_space_dims_expected


@pytest.mark.skip("TODO")
def test_initial_obs(env):
    obs = env.reset()
    if env._observable_object_pos:
        obs_space_dims_expected = 19
    else:
        obs_space_dims_expected = 16
    assert len(obs) == obs_space_dims_expected


@pytest.mark.skip("TODO")
def test_eps_done(env):
    if env._observable_object_pos:
        obs_space_dims_expected = 19
    else:
        obs_space_dims_expected = 16

    done = False
    for t in range(10):
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        assert len(obs) == obs_space_dims_expected
        assert isinstance(reward, float)
        assert isinstance(done, bool)
        if done:
            obs = env.reset()
