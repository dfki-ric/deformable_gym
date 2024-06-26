from abc import ABC
from typing import Optional

import numpy as np
import pybullet as pb

from ..envs.base_env import BaseBulletEnv, GraspDeformableMixin
from ..envs.sampler import FixedSampler, Sampler
from ..objects.bullet_object import ObjectFactory


class GraspEnv(BaseBulletEnv, GraspDeformableMixin, ABC):
    INITIAL_POSE = np.r_[
        0.03, -0.025, 1.0, pb.getQuaternionFromEuler([-np.pi / 8, np.pi, 0])
    ]

    def __init__(
        self,
        object_name: str = "insole",
        object_scale: float = 1.0,
        observable_object_pos: bool = False,
        initial_state_sampler: Optional[Sampler] = None,
        **kwargs,
    ):
        self.object_name = object_name
        self.object_scale = object_scale
        self._observable_object_pos = observable_object_pos
        if initial_state_sampler is None:
            self.initial_pose_sampler = FixedSampler(self.INITIAL_POSE)
        else:
            self.initial_pose_sampler = initial_state_sampler

        super().__init__(soft=True, **kwargs)

        # TODO: adapt if non-robot observable objects in environment
        # self.action_space = self.robot.action_space
        # self.observation_space = self.robot.observation_space

    def _load_objects(self):
        super()._load_objects()
        (
            self.object_to_grasp,
            self.object_position,
            self.object_orientation,
        ) = ObjectFactory(self.pb_client).create(self.object_name)

    def reset(self, seed=None, options=None):

        initial_pose = self.initial_pose_sampler.sample_initial_pose()
        self.robot.reset_base(initial_pose)

        self.object_to_grasp.reset()
        self.robot.activate_motors()

        return super().reset(seed, options)

    def _is_truncated(self, state, action, next_state):
        return self._deformable_is_exploded()

    def calculate_reward(self, state, action, next_state, terminated):
        """
        Calculates the reward by counting how many insole vertices are in the
        target position.
        """
        if terminated:
            self.robot.deactivate_motors()
            # remove insole anchors and simulate steps
            self.object_to_grasp.remove_anchors()
            for _ in range(50):
                if self._deformable_is_exploded():
                    return -1.0
                self.simulation.step_to_trigger("time_step")
            height = self.object_to_grasp.get_pose()[2]
            if height < 0.9:
                return -1.0
            else:
                return 1.0
        else:
            return 0.0
