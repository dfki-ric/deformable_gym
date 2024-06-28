import gymnasium

from deformable_gym.envs.floating_mia_grasp_env import FloatingMiaGraspEnv

"""
================================================================
Floating Mia Example
================================================================

This is an example of how to use the FloatingMiaGraspInsole environment. A
random policy is then used to generate ten episodes.
"""

env = gymnasium.make("FloatingMiaGraspInsole-v0", render_mode="human")

obs, info = env.reset()
episode_return = 0
num_episodes = 0

while num_episodes < 10:

    action = env.action_space.sample()

    obs, reward, terminated, truncated, _ = env.step(action)
    episode_return += reward

    if terminated or truncated:
        print(f"Episode finished with return {episode_return}!")
        num_episodes += 1
        episode_return = 0

        obs, _ = env.reset()

env.close()
