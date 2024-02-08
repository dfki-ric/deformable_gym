import gymnasium

from deformable_gym.envs.floating_mia_grasp_env import FloatingMiaGraspEnv
"""
=========
Floating Mia Example
=========

This is an example of how to use the FloatingMiaGraspEnv. A random policy is 
then used to generate ten episodes.

"""

env = gymnasium.make("FloatingMiaGraspInsole-v0", gui=False)
env2 = gymnasium.make("FloatingMiaGraspInsole-v0", gui=True)

print(env.simulation._client)
print(env2.simulation._client)

obs, info = env.reset()
episode_return = 0
num_episodes = 0

obs2, info2 = env2.reset()
episode_return2 = 0
num_episodes2 = 0

while num_episodes <= 2:

    action = env.action_space.sample()
    action2 = env2.action_space.sample()

    obs, reward, terminated, truncated, _ = env.step(action)
    obs2, reward2, terminated2, truncated2, _ = env.step(action)
    episode_return += reward
    episode_return2 += reward2

    if terminated or truncated:
        print(f"Episode finished with return {episode_return} in env!")
        num_episodes += 1
        episode_return = 0

        env.reset()

    if terminated2 or truncated2:
        print(f"Episode finished with return {episode_return2} in env2!")
        num_episodes2 += 1
        episode_return2 = 0

        env2.reset()

env.close()
env2.close()
