from argparse import ArgumentParser

import gymnasium as gym

from deformable_gym.envs.mujoco.grasp_env import GraspEnv

robot_name2id = {
    "shadow_hand": "FloatingShadow",
    "mia_hand": "FloatingMia",
    "ur5_mia": "UR5Mia",
    "ur10_mia": "UR10Mia",
    "ur10ft_mia": "UR10FTMia",
    "ur10e_mia": "UR10EMia",
    "ur5_shadow": "UR5Shadow",
    "ur10_shadow": "UR10Shadow",
    "ur10e_shadow": "UR10EShadow",
}
obj_name2id = {
    "insole_fixed": "Insole",
    "pillow_fixed": "Pillow",
}


def make_env():
    parser = ArgumentParser()
    parser.add_argument(
        "--robot",
        type=str,
        default="mia_hand",
        help=f"available robots: {list(robot_name2id.keys())}",
    )
    parser.add_argument(
        "--obj",
        type=str,
        default="insole_fixed",
        help=f"available objects: {list(obj_name2id.keys())}",
    )
    parser.add_argument(
        "--control",
        type=str,
        default="joint",
        help="available control types: ['joint', 'mocap']",
    )
    args = parser.parse_args()
    robot_id = robot_name2id[args.robot]
    obj_id = obj_name2id[args.obj]
    env_id = f"Mj{robot_id}Grasp{obj_id}-v0"
    control_type = args.control
    env = gym.make(env_id, control_type=control_type, render_mode="human")
    return env


if __name__ == "__main__":
    env = make_env()
    episode = 0
    n_episodes = 3
    observation, info = env.reset()
    while episode < n_episodes:
        action = env.action_space.sample()
        observation, reward, terminate, truncated, info = env.step(action)
        if terminate:
            episode += 1
            print(f"Episode {episode} finished with reward {reward}")
            observation, info = env.reset()
    env.close()
