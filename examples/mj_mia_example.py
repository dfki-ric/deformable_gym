import gymnasium as gym

from deformable_gym.envs.mujoco.mia_grasp import MiaHandGrasp

if __name__ == "__main__":
    env = gym.make("MjMiaGraspInsole-v0")
    episode = 0
    n_episodes = 3
    observation, info = env.reset()
    env.render()
    while episode < n_episodes:
        action = env.action_space.sample()
        observation, reward, terminate, truncated, info = env.step(action)
        env.render()
        if terminate:
            episode += 1
            print(f"Episode {episode} finished with reward {reward}")
            observation, info = env.reset()
    env.close()
