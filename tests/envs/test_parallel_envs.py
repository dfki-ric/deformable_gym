import gymnasium

SEED = 0


def test_parallel_envs():
    env = gymnasium.make("FloatingMiaGraspInsole-v0", horizon=10)
    env2 = gymnasium.make("FloatingMiaGraspInsole-v0", horizon=10)

    obs, info = env.reset(seed=SEED)
    num_steps = 0
    episode_return = 0
    num_episodes = 0

    obs2, info2 = env2.reset(seed=SEED)
    num_steps2 = 0
    episode_return2 = 0
    num_episodes2 = 0

    print(f"{obs=}")

    while num_episodes < 3 and num_episodes2 < 3:

        action = env.action_space.sample()
        action2 = env2.action_space.sample()

        obs, reward, terminated, truncated, _ = env.step(action)
        num_steps += 1
        obs2, reward2, terminated2, truncated2, _ = env2.step(action2)
        num_steps2 += 1
        episode_return += reward
        episode_return2 += reward2

        if terminated or truncated:
            assert episode_return == -1
            num_episodes += 1
            episode_return = 0
            env.reset()

        if terminated2 or truncated2:
            assert episode_return2 == -1
            num_episodes2 += 1
            episode_return2 = 0
            env2.reset()

    env.close()
    env2.close()

    assert num_steps == num_steps2
