import gymnasium

from deformable_gym.envs.floating_mia_grasp_env import FloatingMiaGraspEnv

SEED = 0

env = gymnasium.make("FloatingMiaGraspInsole-v0")
env2 = gymnasium.make("FloatingMiaGraspInsole-v0")

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
        num_episodes += 1
        episode_return = 0
        env.reset()

    if terminated2 or truncated2:
        num_episodes2 += 1
        episode_return2 = 0
        env2.reset()

env.close()
env2.close()
