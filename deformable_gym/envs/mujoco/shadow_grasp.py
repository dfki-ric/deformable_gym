import logging
from typing import Dict, Tuple

import mujoco
import mujoco.viewer
import numpy as np
from gymnasium import spaces
from numpy.typing import ArrayLike, NDArray

from ...helpers import mj_utils as mju
from .base_mjenv import BaseMJEnv

logger = logging.getLogger("SHADOW_GRASP")


class ShadowHandGrasp(BaseMJEnv):

    def __init__(
        self,
        model_path: str,
        robot_path: str,
        object_name: str,
        object_path: str,
        init_frame: str = None,
        max_sim_time: float = 5,
        gui: bool = True,
    ):
        super().__init__(
            model_path,
            robot_path,
            object_name,
            object_path,
            init_frame,
            max_sim_time,
            gui,
        )
        self.observation_space = self._get_observation_space()
        self.action_space = self._get_action_space()
        self.reward_range = (-1, 1)

    def _get_action_space(self) -> spaces.Box:
        n_act = self.robot.nact
        low = self.robot.ctrl_range[:, 0].copy()
        high = self.robot.ctrl_range[:, 1].copy()
        return spaces.Box(low=low, high=high, shape=(n_act,), dtype=np.float64)

    def _get_observation_space(self) -> spaces.Box:
        nq = self.robot.nq
        low = -np.inf  # TODO: joint space range
        high = np.inf
        return spaces.Box(low=low, high=high, shape=(nq + 3,), dtype=np.float64)

    def reset(self, *, seed=None, options=None) -> Tuple[NDArray, Dict]:
        super().reset(seed=seed, options=options)
        observation = self._get_observation()
        info = self._get_info()
        return observation, info

    def _get_observation(self) -> NDArray:
        # TODO: end effector position for a single hand?
        robot_qpos = self.robot.get_qpos(self.data)
        obj_pos = self.object.get_com(self.data)
        return np.concatenate([robot_qpos, obj_pos])

    def _step_simulation(self, time: float) -> None:
        """Step Mujoco Simulation for a given time (unit: second).

        Args:
            time (float): simulation time in seconds
        """
        start_time = self.data.time
        while self.data.time - start_time < time:
            mujoco.mj_step(self.model, self.data)
            if self.gui:
                self.viewer.sync()

    def _get_reward(self, terminated: bool) -> int:
        """Calculate reward by removing the platform and check if object falls to the ground.
        0 reward: max_sim_time is not reached yet
        -1 reward: object falls to the ground after removing the platform
        1 reward: object is grasped successfully by the robot hand

        Args:
            terminated (bool): if episode is terminated

        Returns:
            int: reward gotten per step
        """
        # TODO: set a proper threshold for simulation time and height of the object
        if not terminated:
            return 0
        mju.remove_geom(self.model, self.data, "platform")
        self._step_simulation(7)
        obj_hight = self.object.get_com(self.data)[2]
        logger.debug(f"Object hight: {obj_hight}")
        if obj_hight > 0.2:
            return 1
        else:
            return -1

    def _is_terminated(self, sim_time: float) -> bool:
        return sim_time >= self.max_sim_time

    def step(self, ctrl: ArrayLike) -> Tuple[NDArray, int, bool, bool, Dict]:
        sim_time = self.data.time
        self.data.ctrl[:] = ctrl
        mujoco.mj_step(self.model, self.data)

        observation = self._get_observation()
        terminated = self._is_terminated(sim_time)
        truncated = self._is_truncated()
        reward = self._get_reward(terminated)
        info = self._get_info()
        return observation, reward, terminated, truncated, info
