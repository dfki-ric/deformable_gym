from time import perf_counter
from typing import Dict, Tuple

import mujoco
import mujoco.viewer
import numpy as np
from gymnasium import spaces
from numpy.typing import ArrayLike, NDArray

from ...helpers import mj_utils as mju
from .base_mjenv import BaseMJEnv


class ShadowHandGrasp(BaseMJEnv):

    def __init__(
        self,
        model_path: str,
        robot_path: str,
        object_path: str,
        max_sim_time: float,
        gui: bool = True,
    ):
        super().__init__(model_path, robot_path, object_path, max_sim_time, gui)
        self.observation_space = self._get_observation_space()
        self.action_space = self._get_action_space()
        self.reward_range = (-1, 1)

    def _get_action_space(self) -> spaces.Box:
        n_act = self.robot.nact
        low = self.robot.ctrl_range[:, 0].copy()
        high = self.robot.ctrl_range[:, 1].copy()
        return spaces.Box(low=low, high=high, shape=(n_act,), dtype=np.float64)

    def _get_observation_space(self) -> spaces.Box:
        dof = self.robot.dof
        low = -np.inf
        high = np.inf
        return spaces.Box(low=low, high=high, shape=(dof,), dtype=np.float64)

    def reset(self, *, seed=None, options=None) -> Tuple[NDArray, Dict]:
        super().reset(seed=seed, options=options)
        self.model, _ = mju.load_model(self.model_path)
        mujoco.mj_resetData(self.model, self.data)
        self.robot.reset(self.model, self.data)
        observation = self._get_observation()
        info = self._get_info()
        return observation, info

    def _get_observation(self) -> NDArray:
        robot_qpos = self.robot.get_qpos(self.data)
        return robot_qpos

    def _step_simulation(self, time: float) -> None:
        start_time = perf_counter()
        while perf_counter() - start_time < time:
            mujoco.mj_step(self.model, self.data)
            if self.gui:
                self.viewer.sync()

    def _get_reward(self, terminated: bool) -> int:
        if not terminated:
            return 0
        mju.remove_geom(self.model, self.data, "platform")
        print("Platform removed")
        self._step_simulation(5)
        obj_hight = mju.get_body_com(self.model, self.object.name)[2]
        if obj_hight > 0.1:
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
