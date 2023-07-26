from deformable_gym.envs.floating_shadow_grasp_env import FloatingShadowGraspEnv

"""
=========
Floating Shadow Example
=========

This is an example of how to use the FloatingShadowGraspEnv. A random policy is then 
used to generate ten episodes. 

"""

env = FloatingShadowGraspEnv(
        gui=True,
        horizon=100,
        object_name="insole",
        early_episode_termination=False
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
