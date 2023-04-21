from deformable_gym.envs.floating_shadow_grasp_env import FloatingShadowGraspEnv

env = FloatingShadowGraspEnv(
        gui=True,
        verbose=True,
        horizon=100,
        object_name="insole",
        early_episode_termination=False
)


state = env.reset()
episode_return = 0

while True:

    action = env.action_space.sample()

    state, reward, done, _ = env.step(action)
    episode_return += reward

    if done:
        print(f"Episode finished with return {episode_return}!")
        break
