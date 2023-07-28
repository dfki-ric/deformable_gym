import numpy as np

from abc import ABC
from base_env import BaseBulletEnv
from ..objects.bullet_object import ObjectFactory
from gym.spaces import Box


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

        limits = self.robot.get_joint_limits(self.robot.motors.values())

        if self._observable_object_pos:
            lower_observations = np.append(lower_observations, np.ones(3))
            upper_observations = np.append(upper_observations, -np.ones(3))

        if self._observable_time_step:
            lower_observations = np.append(lower_observations, 0)
            upper_observations = np.append(upper_observations, self.horizon)

        self.observation_space = Box(low=lower_observations, high=upper_observations)

        # build the action space
        lower = [-np.ones(3) * .0005,  # max negative base pos offset
                 -np.ones(4) * .000005,  # max negative base orn offset
                 limits[0][self.actuated_finger_ids]]  # negative joint limits

        upper = [np.ones(3) * .0005,  # max positive base pos offset
                 np.ones(4) * .000005,  # max positive base orn offset
                 limits[1][self.actuated_finger_ids]]  # positive joint limits

        self.action_space = self.robot.action_space

    def _load_objects(self):
        super()._load_objects()
        self.object_to_grasp, self.object_position, self.object_orientation = ObjectFactory().create(self.object_name)

    def reset(self, hard_reset=False, pos=None):

        self.robot.reset(pos)
        self.object_to_grasp.reset()
        self.robot.activate_motors()

        return super().reset()

    def observe_state(self):
        joint_pos = self.robot.get_joint_positions(self.robot.actuated_real_joints)
        ee_pose = self.robot.get_ee_pose()
        sensor_readings = self.robot.get_sensor_readings()

        state = np.concatenate([ee_pose, joint_pos, sensor_readings])

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
