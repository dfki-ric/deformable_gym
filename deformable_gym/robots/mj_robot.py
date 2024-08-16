from abc import ABC
from typing import List

import mujoco
import numpy as np
from numpy.typing import ArrayLike, NDArray

from ..envs.mujoco.asset_manager import AssetManager
from ..helpers import mj_utils as mju
from ..helpers.mj_utils import Pose


class MJRobot(ABC):
    """
    Abstract base class for creating robot instances in a MuJoCo simulation environment.

    This class provides a set of methods to manage robot configurations, including
    initialization, accessing joint and actuator information, and setting robot poses and controls.

    Attributes:
        init_pos (dict): A dictionary that can store initial positions for different robot models.
                         This can be overridden by subclasses to provide specific initial positions.
        name (str): The name of the robot model.
        model (mujoco.MjModel): The MuJoCo model of the robot, loaded from the corresponding XML file.
        nq (int): Number of generalized coordinates (joints) in the robot model.
        dof (int): Degrees of freedom in the robot model.
        nact (int): Number of actuators in the robot model.
        ctrl_range (NDArray): The control range for each actuator, indicating the min and max values.
        joints (List[str]): A list of joint names for the robot.
        actuators (List[str]): A list of actuator names for the robot.
    """

    init_pos = {}

    def __init__(self, name: str) -> None:

        self.name = name
        self.model = AssetManager().load_asset(self.name)
        self.nq = self.model.nq
        self.dof = self.model.nv
        self.nact = self.model.nu
        self.ctrl_range = self.model.actuator_ctrlrange.copy()

    @property
    def joints(self) -> List[str]:
        return mju.get_joint_names(self.model)

    @property
    def actuators(self) -> List[str]:
        return mju.get_actuator_names(self.model)

    def get_qpos(self, model: mujoco.MjModel, data: mujoco.MjData) -> NDArray:
        """
        Get the current joint positions (qpos) for the robot.

        Args:
            model (mujoco.MjModel): The MuJoCo model object containing the robot's configuration.
            data (mujoco.MjData): The MuJoCo data object containing the current simulation state.

        Returns:
            NDArray: An array of joint positions for the robot.
        """

        return mju.get_joint_qpos(model, data, *self.joints)

    def get_qvel(self, model: mujoco.MjModel, data: mujoco.MjData) -> NDArray:
        """
        Get the current joint velocities (qvel) for the robot.

        Args:
            model (mujoco.MjModel): The MuJoCo model object containing the robot's configuration.
            data (mujoco.MjData): The MuJoCo data object containing the current simulation state.

        Returns:
            NDArray: An array of joint velocities for the robot.
        """

        return mju.get_joint_qvel(model, data, *self.joints)

    def set_pose(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        pose: Pose,
    ) -> None:
        """
        Set the pose (position and orientation) of the robot's base.

        This method sets the position and orientation of the robot's base body in the simulation,
        and then recalculates the simulation state using `mujoco.mj_forward`.

        Args:
            model (mujoco.MjModel): The MuJoCo model object containing the robot's configuration.
            data (mujoco.MjData): The MuJoCo data object containing the current simulation state.
            pose (Pose): A Pose object containing the desired position and orientation for the robot's base.
        """

        model.body(self.name).pos[:] = pose.position
        model.body(self.name).quat[:] = pose.orientation
        mujoco.mj_forward(model, data)

    def set_ctrl(
        self, model: mujoco.MjModel, data: mujoco.MjData, ctrl: ArrayLike
    ) -> None:
        """
        Apply control inputs to the robot's actuators.

        This method sets the control input for each actuator based on the provided control vector.
        The control vector must have a length equal to the number of actuators.

        Args:
            model (mujoco.MjModel): The MuJoCo model object containing the robot's configuration.
            data (mujoco.MjData): The MuJoCo data object containing the current simulation state.
            ctrl (ArrayLike): An array of control inputs for the actuators.

        Raises:
            AssertionError: If the length of the control vector does not match the number of actuators.
        """

        assert (
            len(ctrl) == self.nact
        ), f"Control vector should have length {self.nact}"
        for i, act in enumerate(self.actuators):
            mju.set_actuator_ctrl(model, data, act, ctrl[i])


class ShadowHand(MJRobot):

    init_pose = {
        "insole_fixed": Pose(
            [-0.35, 0.00, 0.49], mju.euler2quat([0, np.pi / 2, 0])
        ),
    }

    def __init__(self) -> None:
        super().__init__("shadow_hand")


class MiaHand(MJRobot):
    mrl_actuators = [
        "j_middle_fle_A",
        "j_ring_fle_A",
        "j_little_fle_A",
    ]
    mrl_joints = ["j_middle_fle", "j_ring_fle", "j_little_fle"]

    init_pose = {
        "insole_fixed": Pose(
            [-0.1, 0, 0.49], mju.euler2quat([0, np.pi, np.pi / 2])
        ),
    }

    def __init__(self) -> None:
        super().__init__("mia_hand")
        self.nact = self.model.nu - 2
        self.ctrl_range = np.take(
            self.model.actuator_ctrlrange, [0, 1, 2, 5, 6, 7, 8, 9, 10], axis=0
        )

    @property
    def actuators(self):
        all_actuators = mju.get_actuator_names(self.model)
        acts = [act for act in all_actuators if act not in self.mrl_actuators]
        acts.append("j_mrl_fle_A")
        return acts

    def _set_mrl_ctrl(
        self, model: mujoco.MjModel, data: mujoco.MjData, ctrl: float
    ) -> None:
        for act in self.mrl_actuators:
            mju.set_actuator_ctrl(model, data, act, ctrl)

    def set_ctrl(
        self, model: mujoco.MjModel, data: mujoco.MjData, ctrl: ArrayLike
    ) -> None:
        assert (
            len(ctrl) == self.nact
        ), f"Control vector should have length {self.nact}, now it is {len(ctrl)}"
        for i, act in enumerate(self.actuators):
            if act == "j_mrl_fle_A":
                self._set_mrl_ctrl(model, data, ctrl[i])
            else:
                mju.set_actuator_ctrl(model, data, act, ctrl[i])


class RobotFactory:

    @staticmethod
    def create(name: str) -> MJRobot:
        if name == "shadow_hand":
            return ShadowHand()
        elif name == "mia_hand":
            return MiaHand()
        else:
            raise ValueError(f"Robot {name} not found.")
