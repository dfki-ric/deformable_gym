from typing import Dict, Optional, Tuple

import mujoco
import mujoco.viewer
import numpy as np
from numpy.typing import ArrayLike, NDArray

from ...helpers import mj_utils as mju
from .base_mjenv import BaseMJEnv


class GraspEnv(BaseMJEnv):
    """
    A custom MuJoCo environment for a grasping task, where a robot attempts to grasp an object.
    """

    def __init__(
        self,
        robot_name: str,
        obj_name: str,
        observable_object_pos: bool = True,
        max_sim_time: float = 5,
        gui: bool = True,
        init_frame: Optional[str] = None,
        **kwargs,
    ):
        super().__init__(
            robot_name,
            obj_name,
            observable_object_pos,
            max_sim_time,
            gui,
            init_frame,
            **kwargs,
        )

        self.reward_range = (-1, 1)

    def reset(self, *, seed=None, options=None) -> Tuple[NDArray, Dict]:
        super().reset(seed=seed, options=options)
        observation = self._get_observation()
        info = self._get_info()
        return observation, info

    def _get_observation(self) -> NDArray:
        """
        The observation includes the robot's joint qpos in their generalized coordinates,
        and optionally, the object's position.

        Returns:
            NDArray: A numpy array containing the observation.
        """
        robot_qpos = self.robot.get_qpos(self.model, self.data)
        if self.observable_object_pos:
            obj_pos = self.object.get_center_of_mass(self.data)
            obs = np.concatenate([robot_qpos, obj_pos])
        else:
            obs = robot_qpos
        return obs

    def _pause_simulation(self, time: float) -> None:
        """
        Steps the MuJoCo simulation for a specified amount of time.

        certain constraints of object will be disabled and all joints will be freezed
        before stepping the simulation to allow the robot to grasp the object without
        external influences.

        Args:
            time (float): The duration in seconds for which to step the simulation.
        """
        mju.disable_equality_constraint(
            self.model, self.data, *self.object.eq_constraints_to_disable
        )
        mju.disable_joint(self.model, self.data, *self.robot.joints)
        start_time = self.data.time
        while self.data.time - start_time < time:
            mujoco.mj_step(self.model, self.data)
            if self.gui:
                self.render()

    def _get_reward(self, terminated: bool) -> int:
        """
        Calculates the reward based on the robot's success in grasping the object.

        The reward is calculated after removing all fixed constraints and checking
        if the object remains grasped by the robot.

        Args:
            terminated (bool): Whether the episode has terminated.

        Returns:
            int: The reward for the current step. Possible values are:
                 - 0: The episode has not yet terminated.
                 - 1: The object is successfully grasped by the robot.
                 - -1: The object falls to the ground.
        """

        if not terminated:
            return 0
        self._pause_simulation(1)
        obj_height = self.object.get_center_of_mass(self.data)[2]
        if obj_height > 0.2:
            return 1
        else:
            return -1

    def _is_terminated(self, sim_time: float) -> bool:
        """
        Determines whether the episode has terminated based on the simulation time.

        Args:
            sim_time (float): The current simulation time.

        Returns:
            bool: True if the simulation time has exceeded the maximum allowed time, otherwise False.
        """
        return sim_time >= self.max_sim_time

    def _is_truncated(self) -> bool:
        return False

    def _get_info(self) -> Dict:
        """
        If the GUI viewer is running, this method will return a dictionary indicating that.

        Returns:
            Dict: A dictionary containing information about the environment.
        """
        if self.gui and self.viewer is not None:
            return {"is_viewer_running": self.viewer.is_running()}
        return {}

    def step(self, action: ArrayLike) -> Tuple[NDArray, int, bool, bool, Dict]:
        """
        Advances the simulation applying the given control input to the robot

        Args:
            ctrl (ArrayLike): The control input to be applied to the robot.

        Returns:
            Tuple[NDArray, int, bool, bool, Dict]: observation, reward, termination flag,
                                                truncation flag, and an info.
        """
        sim_time = self.data.time
        self.robot.set_ctrl(self.model, self.data, action)
        mujoco.mj_step(self.model, self.data)

        observation = self._get_observation()
        terminated = self._is_terminated(sim_time)
        truncated = self._is_truncated()
        reward = self._get_reward(terminated)
        info = self._get_info()
        return observation, reward, terminated, truncated, info
