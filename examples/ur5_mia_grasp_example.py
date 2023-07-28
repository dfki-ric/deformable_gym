from deformable_gym.envs.ur5_mia_grasp_env import UR5MiaGraspEnv

"""
=========
Floating Mia Example
=========

This is an example of how to use the FloatingMiaGraspEnv. A random policy is then
used to generate ten episodes.

"""

env = UR5MiaGraspEnv(
        gui=True,
        object_name="insole2"
)

env.reset()
episode_return = 0
num_episodes = 0

while num_episodes <= 10:

    action = env.action_space.sample()

    state, reward, done, _ = env.step(action)
    episode_return += reward

    if done:
        print(f"Episode finished with return {episode_return}!")
        num_episodes += 1
