from deformable_gym.envs.ur10_shadow_grasp_env import UR10ShadowGraspEnv

"""
=========
Floating Mia Example
=========

This is an example of how to use the FloatingMiaGraspEnv. A random policy is then
used to generate ten episodes.

"""

env = UR10ShadowGraspEnv(gui=True, object_name="insole")

env.reset()
episode_return = 0
num_episodes = 0

while num_episodes <= 10:

    action = env.action_space.sample()

    state, reward, terminated, truncated, _ = env.step(action)
    episode_return += reward

    if terminated or truncated:
        print(f"Episode finished with return {episode_return}!")
        num_episodes += 1
        episode_return = 0

        env.reset()
