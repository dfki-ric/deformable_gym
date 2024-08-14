from typing import Dict, Optional, Tuple

import mujoco
import mujoco.viewer
import numpy as np
from numpy.typing import ArrayLike, NDArray

from ...helpers import mj_utils as mju
from .base_mjenv import BaseMJEnv


class GraspEnv(BaseMJEnv):

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
        self.robot.set_pose(
            self.model, self.data, self.robot.init_pose[self.object.name]
        )
        observation = self._get_observation()
        info = self._get_info()
        return observation, info

    def _get_observation(self) -> NDArray:
        robot_qpos = self.robot.get_qpos(self.model, self.data)
        if self.observable_object_pos:
            obj_pos = self.object.get_current_com(self.data)
            obs = np.concatenate([robot_qpos, obj_pos])
        else:
            obs = robot_qpos
        return obs

    def _pause_simulation(self, time: float) -> None:
        """Step Mujoco Simulation for a given time.

        Args:
            time (float): simulation time in seconds
        """
        # mju.remove_geom(self.model, self.data, "platform") # might be off here...
        self.object.disable_eq_constraint(self.model, self.data)
        mju.disable_joint(self.model, self.data, *self.robot.joints)
        start_time = self.data.time
        while self.data.time - start_time < time:
            mujoco.mj_step(self.model, self.data)
            if self.gui:
                self.render()

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
        if not terminated:
            return 0
        self._pause_simulation(1)
        obj_hight = self.object.get_current_com(self.data)[2]
        if obj_hight > 0.2:
            return 1
        else:
            return -1

    def _is_terminated(self, sim_time: float) -> bool:
        return sim_time >= self.max_sim_time

    def _is_truncated(self) -> bool:
        return False

    def _get_info(self) -> Dict:
        if self.viewer is not None:
            return {"is_viewer_running": self.viewer.is_running()}
        return {}

    def step(self, ctrl: ArrayLike) -> Tuple[NDArray, int, bool, bool, Dict]:
        sim_time = self.data.time
        self.robot.set_ctrl(self.model, self.data, ctrl)
        mujoco.mj_step(self.model, self.data)

        observation = self._get_observation()
        terminated = self._is_terminated(sim_time)
        truncated = self._is_truncated()
        reward = self._get_reward(terminated)
        info = self._get_info()
        return observation, reward, terminated, truncated, info
