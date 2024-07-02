from abc import ABC, abstractmethod
from typing import Any, Optional, Tuple, Union

import gymnasium as gym
import mujoco
import mujoco.viewer
import numpy as np
from numpy.typing import ArrayLike, NDArray

from ...helpers import mj_utils as mju
from ...objects.mj_object import MJObject
from ...robots.mj_robot import MJRobot


class BaseMJEnv(gym.Env, ABC):

    def __init__(
        self,
        model_path: str,
        robot_path: str,
        object_path: str,
        max_sim_time: float,
        gui: bool = True,
    ):
        self.model_path = model_path
        self.robot = MJRobot(robot_path)
        self.object = MJObject(object_path)
        self.max_sim_time = max_sim_time
        self.model, self.data = mju.load_model(model_path)
        self.gui = gui
        self.viewer = None

    def set_state(self, q_pos: ArrayLike, q_vel: ArrayLike) -> None:
        """
        Set the state of the MuJoCo simulation.
        """
        self.data.qpos[:] = q_pos
        self.data.qvel[:] = q_vel
        mujoco.mj_forward(self.model, self.data)

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ) -> tuple[NDArray[np.float64], dict[str, Any]]:
        """
        Reset the environment to the initial state.
        """
        super().reset(seed=seed, options=options)

    @abstractmethod
    def _get_observation(self):
        pass

    @abstractmethod
    def _get_reward(self):
        pass

    @abstractmethod
    def _is_terminated(self):
        pass

    def _is_truncated(self):
        return False

    def _get_info(self):
        return {}

    @abstractmethod
    def step(
        self, ctrl: NDArray[np.float64]
    ) -> Tuple[NDArray[np.float64], float, bool, bool, dict]:
        """
        Step the environment forward using the given control input.
        """
        pass

    def render(self) -> None:
        """
        Render a frame from the MuJoCo simulation as specified by the render_mode.
        """
        assert self.gui, "GUI is not enabled"
        if self.viewer is not None:
            self.viewer.sync()
        else:
            self.viewer = mujoco.viewer.launch_passive(
                self.model, self.data, show_left_ui=False, show_right_ui=False
            )

    def close(self) -> None:
        """
        Close the environment.
        """
        if self.viewer is not None:
            self.viewer.close()
