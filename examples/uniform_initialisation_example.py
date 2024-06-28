import gymnasium

from deformable_gym.envs.floating_mia_grasp_env import FloatingMiaGraspEnv
from deformable_gym.envs.sampler import UniformSampler

base_initial_pose = FloatingMiaGraspEnv.INITIAL_POSE.copy()
low = base_initial_pose.copy()
high = base_initial_pose.copy()

low[:3] -= 0.03
high[:3] += 0.03

sampler = UniformSampler(low, high, seed=0)

env = gymnasium.make(
    "FloatingMiaGraspInsole-v0",
    initial_state_sampler=sampler,
    render_mode="human",
)

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
