from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Dict, Tuple

import gymnasium as gym
import mujoco
import mujoco.viewer
import numpy as np
from gymnasium import spaces
from gymnasium.envs.mujoco.mujoco_rendering import MujocoRenderer
from numpy.typing import ArrayLike, NDArray

from ...helpers import asset_manager as am
from ...helpers import mj_utils as mju
from ...helpers.mj_mocap_control import MocapControl
from ...objects.mj_object import ObjectFactory
from ...robots.mj_robot import RobotFactory

MOCAP_POS_CTRL_RANGE = np.array([[-0.005, 0.005]] * 3)
MOCAP_QUAT_CTRL_RANGE = np.array([[-np.pi / 100, np.pi / 100]] * 3)


class BaseMJEnv(gym.Env, ABC):
    """
    Base environment class for simulating robotic tasks using the MuJoCo physics engine.

    This class provides the foundational setup for environments where a robot interacts
    with objects in a simulated scene. It handles the initialization of the robot and object,
    the definition of action and observation spaces, and the integration with the MuJoCo
    simulator for physics-based simulation.

    Attributes:
    -----------
    scene : str
        The XML string representing the MuJoCo scene created by the `AssetManager`.
    model : mujoco.MjModel
        The MuJoCo model used in the simulation.
    data : mujoco.MjData
        MuJoCo data structure holding the simulation state.
    robot : MjRobot
        The robot entity within the environment.
    object : MjObject
        The object entity within the environment.
    observable_object_pos : bool
        Whether the object's position is part of the observation space.
    init_frame : str or None
        Keyframe for initial state setup.
    max_sim_time : float
        Maximum simulation time per episode.
    render_mode : str or None
        The rendering mode (e.g., 'human', 'rgb_array').
    renderer : MujocoRenderer or mujoco.viewer
        Renderer used for visualization, if applicable.
    observation_space : gym.spaces.Box
        Observation space defining the limits of sensor readings or state data available to the agent.
    action_space : gym.spaces.Box
        Action space defining the limits of control actions available to the agent.
    """

    metadata = {"render_modes": ["human", "rgb_array", "depth_array"]}

    def __init__(
        self,
        robot_name: str,
        obj_name: str,
        frame_skip: int = 5,
        observable_object_pos: bool = True,
        control_type: str = "mocap",
        max_sim_time: float = 10,
        render_mode: str | None = None,
        mocap_cfg: dict[str, str] | None = None,
        init_frame: str | None = None,
        default_cam_config: dict[str, Any] | None = None,
        camera_name: str | None = None,
        camera_id: int | None = None,
    ):
        self.scene = am.create_scene(robot_name, obj_name)
        self.model, self.data = mju.load_model_from_string(self.scene)
        self.robot = RobotFactory.create(robot_name, control_type)
        self.object = ObjectFactory.create(obj_name)
        self.frame_skip = frame_skip
        self.observable_object_pos = observable_object_pos
        self.control_type = control_type
        if control_type == "mocap":
            if mocap_cfg is not None:
                self.mocap = MocapControl(**mocap_cfg)
            else:
                self.mocap = MocapControl()
        self.init_frame = init_frame
        self.max_sim_time = max_sim_time
        if render_mode is not None:
            assert render_mode in self.metadata["render_modes"]
        self.render_mode = render_mode
        self.camera_name = camera_name
        self.camera_id = camera_id
        self.renderer = self._get_renderer(default_cam_config)

        self.observation_space = self._get_observation_space()
        self.action_space = self._get_action_space()

    def _get_renderer(self, default_cam_config: dict[str, Any] | None):
        """
        Returns the renderer object for the environment.

        Returns:
            MujocoRenderer or mujoco native viewer or None: The renderer object for the environment.
        """
        if self.render_mode == "human":
            renderer = mujoco.viewer.launch_passive(
                self.model, self.data, show_left_ui=False, show_right_ui=False
            )
            if default_cam_config is not None:
                for attr, value in default_cam_config.items():
                    setattr(renderer.cam, attr, value)
        elif (
            self.render_mode == "rgb_array" or self.render_mode == "depth_array"
        ):
            renderer = MujocoRenderer(self.model, self.data, default_cam_config)
        else:
            renderer = None
        return renderer

    def _get_action_space(self) -> spaces.Box:
        """
        Defines the action space for the environment based on the robot's control range.

        Returns:
            spaces.Box: A continuous space representing the possible actions the agent can take.
        """
        if self.control_type == "mocap":
            shape = self.robot.n_actuator + 6
            low = np.concatenate(
                [
                    MOCAP_POS_CTRL_RANGE[:, 0],
                    MOCAP_QUAT_CTRL_RANGE[:, 0],
                    self.robot.ctrl_range[:, 0],
                ]
            )
            high = np.concatenate(
                [
                    MOCAP_POS_CTRL_RANGE[:, 1],
                    MOCAP_QUAT_CTRL_RANGE[:, 1],
                    self.robot.ctrl_range[:, 1],
                ]
            )
        elif self.control_type == "joint":
            shape = self.robot.n_actuator
            low = self.robot.ctrl_range[:, 0].copy()
            high = self.robot.ctrl_range[:, 1].copy()
        else:
            raise ValueError(f"Unsupported control type: {self.control_type}")
        return spaces.Box(low=low, high=high, shape=(shape,), dtype=np.float64)

    def _get_observation_space(self) -> spaces.Box:
        """
        Defines the observation space for the environment, including the robot's joint positions
        and, optionally, the object's position.

        Returns:
            spaces.Box: A continuous space representing the state observations available to the agent.
        """

        n_qpos = self.robot.n_qpos
        low = self.robot.joint_range[:, 0].copy()
        high = self.robot.joint_range[:, 1].copy()
        if self.observable_object_pos:
            low = np.concatenate([low, [-np.inf] * 3])
            high = np.concatenate([high, [np.inf] * 3])
            return spaces.Box(
                low=low, high=high, shape=(n_qpos + 3,), dtype=np.float64
            )
        return spaces.Box(low=low, high=high, shape=(n_qpos,), dtype=np.float64)

    def reset(
        self,
        *,
        seed: int | None = None,
        options: dict | None = None,
    ) -> tuple[NDArray[np.float64], dict[str, Any]]:
        """
        Resets the environment to its initial state.

        Args:
            seed (Optional[int], optional): A random seed for resetting the environment. Default is None.
            options (Optional[Dict], optional): Additional options for resetting the environment. Default is None.

        Returns:
            tuple: A tuple containing the initial observation and an empty info dictionary.
        """

        super().reset(seed=seed, options=options)
        self.model, _ = mju.load_model_from_string(self.scene)
        mujoco.mj_resetData(self.model, self.data)
        self.robot.set_pose(
            self.model, self.data, self.robot.init_pose.get(self.object.name)
        )
        self.object.set_pose(
            self.model, self.data, self.object.init_pose.get(self.robot.name)
        )
        if self.init_frame is not None:
            self._load_keyframe(self.init_frame)
        if self.control_type == "mocap":
            if not self.mocap.eq_is_active(self.model, self.data):
                self.mocap.enable_eq(self.model, self.data)
            self.mocap.attach_mocap2weld_body(self.model, self.data)

    def _set_state(
        self,
        *,
        qpos: ArrayLike | None = None,
        qvel: ArrayLike | None = None,
    ) -> None:
        if qpos is not None:
            self.data.qpos[:] = qpos
        if qvel is not None:
            self.data.qvel[:] = qvel
        mujoco.mj_forward(self.model, self.data)

    def _load_keyframe(self, frame_name: str):
        """
        Loads a predefined keyframe and sets the environment's state to it.

        Args:
            frame_name (str): The name of the keyframe to load.
        """
        frame = self.model.keyframe(frame_name)
        qpos = frame.qpos.copy()
        qvel = frame.qvel.copy()
        self._set_state(qpos=qpos, qvel=qvel)

    @abstractmethod
    def step(
        self, action: ArrayLike
    ) -> tuple[NDArray[np.float64], float, bool, bool, dict]:
        """
        Step the environment forward using the given control input.
        """
        pass

    def render(self) -> None:
        """
        Render a frame from the MuJoCo simulation as specified by the render_mode.
        """
        if self.render_mode == "human":
            self.renderer.sync()
        elif (
            self.render_mode == "rgb_array" or self.render_mode == "depth_array"
        ):
            return self.renderer.render(self.render_mode)

    def close(self) -> None:
        """
        Close the environment.
        """
        if self.renderer is not None:
            self.renderer.close()
