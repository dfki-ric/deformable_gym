from __future__ import annotations

from typing import Any, Dict, Tuple

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
        frame_skip: int = 3,
        observable_object_pos: bool = True,
        control_type: str = "mocap",
        max_sim_time: float = 6,
        render_mode: str | None = None,
        mocap_cfg: dict[str, str] | None = None,
        init_frame: str | None = None,
        default_cam_config: dict[str, Any] | None = None,
        camera_name: str | None = None,
        camera_id: int | None = None,
    ):
        super().__init__(
            robot_name,
            obj_name,
            frame_skip,
            observable_object_pos,
            control_type,
            max_sim_time,
            render_mode,
            mocap_cfg,
            init_frame,
            default_cam_config,
            camera_name,
            camera_id,
        )

        self.reward_range = (-1, 1)

    def reset(self, *, seed=None, options=None) -> tuple[NDArray, dict]:
        super().reset(seed=seed, options=options)
        observation = self._get_observation()
        info = self._get_info()
        return observation, info

    def _get_observation(self) -> NDArray:
        """
        The observation includes the robot's joint qpos in their generalized coordinates,
        and optionally, the object's position.
        If the render mode is set to "rgb_array" or "depth_array", the observation will be an image.

        Returns:
            NDArray: A numpy array or image containing the observation.
        """
        img = self.render()
        if self.render_mode == "rgb_array" or self.render_mode == "depth_array":
            return img
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
            if self.render_mode == "human":
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

    def _get_info(self) -> dict:
        """
        If the GUI viewer is running, this method will return a dictionary indicating that.

        Returns:
            Dict: A dictionary containing information about the environment.
        """

        return {}

    def step(self, action: ArrayLike) -> tuple[NDArray, int, bool, bool, dict]:
        """
        Advances the simulation applying the given control input to the robot

        Args:
            ctrl (ArrayLike): The control input to be applied to the robot.

        Returns:
            Tuple[NDArray, int, bool, bool, Dict]: observation, reward, termination flag,
                                                truncation flag, and an info.
        """
        sim_time = self.data.time
        if self.control_type == "mocap":
            self.mocap.set_ctrl(self.model, self.data, action[:6])
            self.robot.set_ctrl(self.model, self.data, action[6:])
        else:
            self.robot.set_ctrl(self.model, self.data, action)
        mujoco.mj_step(self.model, self.data, nstep=self.frame_skip)

        observation = self._get_observation()
        terminated = self._is_terminated(sim_time)
        truncated = self._is_truncated()
        reward = self._get_reward(terminated)
        info = self._get_info()
        return observation, reward, terminated, truncated, info
