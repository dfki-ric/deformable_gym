from abc import ABC, abstractmethod
from typing import Any, Optional, Tuple

import gymnasium as gym
import mujoco
import mujoco.viewer
import numpy as np
from gymnasium import spaces
from numpy.typing import ArrayLike, NDArray

from ...helpers import mj_utils as mju
from ...objects.mj_object import ObjectFactory
from ...robots.mj_robot import RobotFactory
from .asset_manager import AssetManager


class BaseMJEnv(gym.Env, ABC):

    def __init__(
        self,
        robot_name: str,
        obj_name: str,
        observable_object_pos: bool = True,
        max_sim_time: float = 5,
        gui: bool = True,
        init_frame: Optional[str] = None,
    ):
        self.scene = self._create_scene(robot_name, obj_name)
        self.model, self.data = mju.load_model_from_string(self.scene)
        self.robot = RobotFactory.create(robot_name)
        self.object = ObjectFactory.create(obj_name)
        self.observable_object_pos = observable_object_pos
        self.init_frame = init_frame
        self.max_sim_time = max_sim_time
        self.gui = gui
        self.viewer = None

        self.observation_space = self._get_observation_space()
        self.action_space = self._get_action_space()

    def _create_scene(self, robot_name: str, obj_name: str) -> str:
        manager = AssetManager()
        return manager.create_scene(robot_name, obj_name)

    def _get_action_space(self) -> spaces.Box:
        n_act = self.robot.nact
        low = self.robot.ctrl_range[:, 0].copy()
        high = self.robot.ctrl_range[:, 1].copy()
        return spaces.Box(low=low, high=high, shape=(n_act,), dtype=np.float64)

    def _get_observation_space(self) -> spaces.Box:
        nq = self.robot.nq
        low = -np.inf  # TODO: joint space range
        high = np.inf
        if self.observable_object_pos:
            return spaces.Box(
                low=low, high=high, shape=(nq + 3,), dtype=np.float64
            )
        return spaces.Box(low=low, high=high, shape=(nq,), dtype=np.float64)

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
        self.model, _ = mju.load_model_from_string(self.scene)
        mujoco.mj_resetData(self.model, self.data)
        self.robot.set_pose(
            self.model, self.data, self.robot.init_pose[self.object.name]
        )
        if self.gui and self.viewer is None:
            self.viewer = mujoco.viewer.launch_passive(
                self.model, self.data, show_left_ui=False, show_right_ui=False
            )
        if self.init_frame is not None:
            self._load_keyframe(self.init_frame)

    def _set_state(
        self,
        *,
        qpos: Optional[ArrayLike] = None,
        qvel: Optional[ArrayLike] = None,
    ) -> None:
        if qpos is not None:
            self.data.qpos[:] = qpos.copy()
        if qvel is not None:
            self.data.qvel[:] = qvel.copy()
        mujoco.mj_forward(self.model, self.data)

    def _load_keyframe(self, frame_name: str):
        frame = self.model.keyframe(frame_name)
        qpos = frame.qpos.copy()
        qvel = frame.qvel.copy()
        self._set_state(qpos=qpos, qvel=qvel)

    @abstractmethod
    def step(
        self, ctrl: ArrayLike
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
        if self.viewer.is_running():
            self.viewer.sync()

    def close(self) -> None:
        """
        Close the environment.
        """
        if self.viewer is not None:
            self.viewer.close()
