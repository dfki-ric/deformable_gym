from deformable_gym.envs.floating_mia_grasp_env import FloatingMiaGraspEnv

env = FloatingMiaGraspEnv(
        gui=True,
        verbose=True,
        horizon=100,
        object_name="insole_on_conveyor_belt/back",
        early_episode_termination=False,
        observable_time_step=False,
        observable_object_pos=True,
        difficulty_mode="hard")


state = env.reset()
episode_return = 0

while True:

    action = env.action_space.sample()

    state, reward, done, _ = env.step(action)
    episode_return += reward

    if done:
        print(f"Episode finished with return {episode_return}!")
        break
