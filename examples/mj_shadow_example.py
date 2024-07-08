import logging

from deformable_gym.envs.mujoco.shadow_grasp import ShadowHandGrasp

logging.basicConfig(level=logging.DEBUG)

model_path = "deformable_gym/envs/mujoco/assets/shadow_grasp/shadow_grasp.xml"
robot_path = "deformable_gym/envs/mujoco/assets/shadow_grasp/shadow_l.xml"
object_name = "insole"
object_path = "deformable_gym/envs/mujoco/assets/shadow_grasp/insole.xml"


if __name__ == "__main__":
    env = ShadowHandGrasp(
        model_path,
        robot_path,
        object_name,
        object_path,
        max_sim_time=0.5,
        gui=True,
    )
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
