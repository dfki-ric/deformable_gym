import numpy as np

from abc import ABC
from base_env import BaseBulletEnv
from ..objects.bullet_object import ObjectFactory


class GraspEnv(BaseBulletEnv, ABC):

    def __init__(self,
                 object_name,
                 observable_object_pos: bool = False,
                 observable_time_step: bool = False):

        self.object_name = object_name
        self._observable_object_pos = observable_object_pos
        self._observable_time_step = observable_time_step

        super().__init__(soft=True)

        self.robot = self._create_robot()

        # TODO: adapt if non-robot observable objects in environment
        self.action_space = self.robot.action_space
        self.observation_space = self.robot.observation_space

    def _load_objects(self):
        super()._load_objects()
        self.object_to_grasp, self.object_position, self.object_orientation = ObjectFactory().create(self.object_name)

    def reset(self, hard_reset=False, pos=None):

        self.robot.reset(pos)
        self.object_to_grasp.reset()
        self.robot.activate_motors()

        return super().reset()

    def _get_observation(self):
        joint_pos = self.robot.get_joint_positions(self.robot.actuated_real_joints)
        sensor_readings = self.robot.get_sensor_readings()

        state = np.concatenate([joint_pos, sensor_readings])

        if self._observable_object_pos:
            obj_pos = self.object_to_grasp.get_pose()[:3]
            state = np.append(state, obj_pos)

        if self._observable_time_step:
            state = np.append(state, self.step_counter)

        return state

    def calculate_reward(self, state, action, next_state, done):
        """
        Calculates the reward by counting how many insole vertices are in the
        target position.
        """
        if done:

            # self.robot.deactivate_motors()
            # remove insole anchors and simulate steps
            self.object_to_grasp.remove_anchors()

            for _ in range(50):
                self.simulation.step_to_trigger("time_step")
            height = self.object_to_grasp.get_pose()[2]
            if height < 0.9:
                return -1.0
            else:
                return 1.0
        else:
            return 0.0
