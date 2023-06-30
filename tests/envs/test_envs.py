import pytest
from deformable_gym.envs.floating_mia_grasp_env import FloatingMiaGraspEnv

env = FloatingMiaGraspEnv(
        gui=False,
        verbose=True,
        horizon=100,
        object_name="insole_on_conveyor_belt/back",
        early_episode_termination=False,
        observable_time_step=False,
        observable_object_pos=True,
        difficulty_mode="hard")

action_space_dims_expected = 10

if env._observable_object_pos == True:
    obs_space_dims_expected = 19
else:
    obs_space_dims_expected = 16

def test_action_space_dims():
    action_space = env.action_space
    print(action_space.shape[0])
    assert action_space.shape[0] == action_space_dims_expected

def test_obs_space_dims():
    obs_space = env.observation_space
    print(obs_space.shape[0])
    assert obs_space.shape[0] == obs_space_dims_expected

def test_initial_obs():
    obs = env.reset()
    assert len(obs) == obs_space_dims_expected

def test_eps_done():
    done = False
    for t in range(10):
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        assert len(obs) == obs_space_dims_expected
        assert isinstance(reward, float)
        assert isinstance(done, bool)
        if done:
            obs = env.reset()
    env.step(action)

def run_tests():
    pytest.main([__file__])

if __name__== '__main__':
    run_tests()
