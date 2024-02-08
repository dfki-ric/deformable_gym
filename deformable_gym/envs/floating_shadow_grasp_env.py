import random

import numpy as np
from gymnasium import spaces
from deformable_gym.robots import shadow_hand
from deformable_gym.envs.base_env import BaseBulletEnv, GraspDeformableMixin
from deformable_gym.helpers import pybullet_helper as pbh
from deformable_gym.objects.bullet_object import ObjectFactory


class FloatingShadowGraspEnv(GraspDeformableMixin, BaseBulletEnv):
    """Grasp an insole with a floating Shadow hand.

    **State space:**
    - End-effector pose: (x, y, z, qx, qy, qz, qw)
    - Finger joint angles: 24 values  TODO: order

    **Action space:**
    - End-effector pose: (x, y, z, qx, qy, qz, qw)
    - Finger joint angle: 3 values

    Parameters
    ----------
    :param gui: Show PyBullet GUI.
    :param real_time: Simulate environment in real time.
    :param verbose: Verbose output.
    :param horizon: Number of steps in simulation.
    :param train: Bool flag indicating if the starting position of the grasped
    object should be sampled from the training or the test set.
    """

    def __init__(self, object_name="insole",
                 horizon=100, train=True,
                 compute_reward=True, object_scale=1.0,
                 observable_object_pos: bool = False,
                 **kwargs):
        self.insole = None
        self.train = train
        self.velocity_commands = False
        self.object_name = object_name
        self.randomised = False
        self.compute_reward = compute_reward
        self.object_scale = object_scale
        self._observable_object_pos = observable_object_pos

        super().__init__(horizon=horizon, soft=True, **kwargs)

        self.hand_world_pose = (0, -0.5, 1, -np.pi/2, np.pi, 0)
        self.robot = self._create_robot()

        limits = pbh.get_limit_array(self.robot.motors.values())

        lower_observations = np.concatenate([
            np.array([-2, -2, 0]), -np.ones(4), limits[0][6:],
            np.array([-10, -10, -10])], axis=0)

        upper_observations = np.concatenate([
            np.array([2, 2, 2]), np.ones(4), limits[1][6:],
            np.array([10, 10, 10])], axis=0)

        if self._observable_object_pos:
            lower_observations = np.append(lower_observations, -np.ones(3)*2)
            upper_observations = np.append(upper_observations, np.ones(3)*2)

        self.observation_space = spaces.Box(
            low=lower_observations, high=upper_observations, dtype=np.float64)

        lower_actions = np.concatenate([
            -np.ones(7)*0.01, limits[0], [0]], axis=0)

        upper_actions = np.concatenate([
            np.ones(7)*0.01, limits[1], [1]], axis=0)

        self.action_space = spaces.Box(low=lower_actions, high=upper_actions, dtype=np.float64)

    def _create_robot(self):
        task_space_limit = None
        orn_limit = None

        if self.velocity_commands:
            robot = shadow_hand.ShadowHandVelocity(
                world_pos=self.hand_world_pose[:3],
                world_orn=self.hand_world_pose[3:],
                task_space_limit=task_space_limit, verbose=self.verbose,
                orn_limit=orn_limit, base_commands=True)
        else:
            robot = shadow_hand.ShadowHandPosition(
                world_pos=self.hand_world_pose[:3],
                world_orn=self.hand_world_pose[3:],
                task_space_limit=task_space_limit, verbose=self.verbose,
                orn_limit=orn_limit, base_commands=True)

        self.simulation.add_robot(robot)

        return robot

    def _load_objects(self):
        super()._load_objects()
        (self.object_to_grasp,
         self.object_position,
         self.object_orientation) = ObjectFactory().create(
            self.object_name,
            pb_client_id=self.simulation.get_physics_client_id())

    def reset(self, seed=None, options=None):
        self.object_to_grasp.reset()
        self.robot.activate_motors()

        return super().reset(seed, options)

    def _is_truncated(self, state, action, next_state):
        return self._deformable_is_exploded()

    def _get_observation(self):
        finger_pos = self.robot.get_joint_positions()[6:]
        return np.concatenate([finger_pos], axis=0)

    def calculate_reward(self, state, action, next_state, done):
        """
        Calculates the reward by counting how many insole vertices are in the
        target position.
        """
        if done:
            if not self.compute_reward:
                return 0.0
            if not (round(action[-1]) == 1 or self.step_counter >= self.horizon):
                return -100

            self.robot.deactivate_motors()
            # remove insole anchors and simulate steps
            self.object_to_grasp.remove_anchors()
            for _ in range(50):
                if self._deformable_is_exploded():
                    return -1
                self.simulation.step_to_trigger("time_step")
            height = self.object_to_grasp.get_pose()[2]
            if height < 0.9:
                return -1
            else:
                return 1
        else:
            return 0.0
