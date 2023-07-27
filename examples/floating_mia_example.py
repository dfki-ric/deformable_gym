from deformable_gym.envs.floating_mia_grasp_env import FloatingMiaGraspEnv

"""
=========
Floating Mia Example
=========

This is an example of how to use the FloatingMiaGraspEnv. A random policy is then
used to generate ten episodes.

"""

env = FloatingMiaGraspEnv(
        gui=True,
        horizon=100,
        object_name="insole_on_conveyor_belt/back",
        observable_time_step=False,
        observable_object_pos=True)

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
