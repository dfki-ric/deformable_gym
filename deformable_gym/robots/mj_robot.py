from __future__ import annotations

from abc import ABC
from typing import List

import mujoco
import numpy as np
from mujoco import mjtJoint
from numpy.typing import ArrayLike, NDArray

from ..envs.mujoco.init_pose import RobotInitPose
from ..helpers import asset_manager as am
from ..helpers import mj_utils as mju
from ..helpers.mj_utils import Pose

SLIDE_CTRL_RANGE = [-0.005, 0.005]
HINGE_CTRL_RANGE = [-np.pi / 180, np.pi / 180]


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
        n_actuator (int): Number of actuators in the robot model.
        ctrl_range (NDArray): The control range for each actuator, indicating the min and max values.
        joints (List[str]): A list of joint names for the robot.
        actuators (List[str]): A list of actuator names for the robot.
    """

    supported_ctrl_types = ["joint", "mocap"]

    def __init__(self, name: str, control_type="joint") -> None:

        self.name = name
        self.model = am.load_asset(self.name)
        if control_type not in self.supported_ctrl_types:
            raise ValueError(
                f"Control type {control_type} not supported.\n Supported control types: {self.supported_ctrl_types}"
            )
        self.control_type = control_type

    @property
    def init_pose(self) -> dict:
        return RobotInitPose.get(self.name)

    @property
    def n_qpos(self) -> int:
        return self.model.nq

    @property
    def dof(self) -> int:
        return self.model.nv

    @property
    def actuators(self) -> list[str]:
        return mju.get_actuator_names(self.model)

    @property
    def n_actuator(self) -> int:
        return len(self.actuators)

    @property
    def joints(self) -> list[str]:
        return mju.get_joint_names(self.model)

    @property
    def joint_range(self) -> NDArray:
        return self.model.jnt_range.copy()

    @property
    def ctrl_range(self) -> NDArray:
        ctrl_range = []
        for actuator in self.actuators:
            joint_id, _ = self.model.actuator(actuator).trnid
            if self.model.joint(joint_id).type == mjtJoint.mjJNT_HINGE:
                ctrl_range.append(HINGE_CTRL_RANGE)
            elif self.model.joint(joint_id).type == mjtJoint.mjJNT_SLIDE:
                ctrl_range.append(SLIDE_CTRL_RANGE)
            else:
                raise ValueError(f"Joint type not supported.")
        return np.array(ctrl_range)

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
        pose: Pose | None,
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
        if pose is not None:
            if pose.position is not None:
                model.body(self.name).pos[:] = pose.position
            if pose.orientation is not None:
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
            len(ctrl) == self.n_actuator
        ), f"Control vector should have length {self.n_actuator}"
        for i, act in enumerate(self.actuators):
            actuator_id = mju.name2id(self.model, act, "actuator")
            new_ctrl = data.ctrl[actuator_id] + ctrl[i]
            mju.set_actuator_ctrl(model, data, act, new_ctrl)


class Ur5(MJRobot):

    def __init__(self, name: str = "ur5", control_type: str = "joint") -> None:
        super().__init__(name, control_type)


class Ur10(MJRobot):

    def __init__(self, name: str = "ur10", control_type: str = "joint") -> None:
        super().__init__(name, control_type)


class Ur10e(MJRobot):

    def __init__(
        self, name: str = "ur10e", control_type: str = "joint"
    ) -> None:
        super().__init__(name, control_type)


class Ur10ft(MJRobot):

    def __init__(
        self, name: str = "ur10ft", control_type: str = "joint"
    ) -> None:
        super().__init__(name, control_type)


class ShadowHand(MJRobot):
    ee_actuators = [
        "ee_A_X",
        "ee_A_Y",
        "ee_A_Z",
        "ee_A_OX",
        "ee_A_OY",
        "ee_A_OZ",
    ]

    def __init__(
        self, name: str = "shadow_hand", control_type: str = "mocap"
    ) -> None:
        super().__init__(name, control_type)
        if self.control_type == "mocap":
            self._excluded_actuators = self.ee_actuators
        else:
            self._excluded_actuators = []

    @property
    def actuators(self):
        all_actuators = mju.get_actuator_names(self.model)

        if self.control_type == "joint":
            return all_actuators
        else:
            return [
                act
                for act in all_actuators
                if act not in self._excluded_actuators
            ]

    @property
    def n_actuator(self):
        return len(self.actuators)


class ShadowHandOnArm(ShadowHand):

    def __init__(
        self, robot_name: str, arm: str, control_type: str = "mocap"
    ) -> None:
        super().__init__(robot_name, control_type)
        self.arm = RobotFactory.create(arm, control_type)
        if self.control_type == "mocap":
            self._excluded_actuators += self.arm.actuators
        else:
            self._excluded_actuators = []


class MiaHand(MJRobot):
    mrl_actuators = [
        "j_middle_fle_A",
        "j_ring_fle_A",
        "j_little_fle_A",
    ]
    mrl_joints = ["j_middle_fle", "j_ring_fle", "j_little_fle"]
    ee_actuators = [
        "ee_A_X",
        "ee_A_Y",
        "ee_A_Z",
        "ee_A_OX",
        "ee_A_OY",
        "ee_A_OZ",
    ]

    def __init__(
        self, name: str = "mia_hand", control_type: str = "mocap"
    ) -> None:
        super().__init__(name, control_type)
        if self.control_type == "mocap":
            self._excluded_actuators = self.mrl_actuators + self.ee_actuators
        else:
            self._excluded_actuators = self.mrl_actuators

    @property
    def actuators(self):
        all_actuators = mju.get_actuator_names(self.model)
        # exclude middle, ring, little actuators
        acts = [
            act for act in all_actuators if act not in self._excluded_actuators
        ]
        # add the combined actuator for middle, ring, little finger control
        acts.append("j_mrl_fle_A")
        return acts

    @property
    def n_actuator(self):
        return len(self.actuators)

    @property
    def ctrl_range(self):

        ctrl_range = []
        mrl_range = HINGE_CTRL_RANGE
        actuators = [
            act
            for act in mju.get_actuator_names(self.model)
            if act not in self._excluded_actuators
        ]
        for actuator in actuators:
            joint_id, _ = self.model.actuator(actuator).trnid
            if self.model.joint(joint_id).type == mjtJoint.mjJNT_HINGE:
                ctrl_range.append(HINGE_CTRL_RANGE)
            elif self.model.joint(joint_id).type == mjtJoint.mjJNT_SLIDE:
                ctrl_range.append(SLIDE_CTRL_RANGE)
            else:
                raise ValueError(f"Joint type not supported.")
        ctrl_range.append(mrl_range)
        return np.array(ctrl_range)

    def _set_mrl_ctrl(
        self, model: mujoco.MjModel, data: mujoco.MjData, ctrl: float
    ) -> None:
        act_id = mju.name2id(self.model, "j_middle_fle_A", "actuator")
        new_ctrl = data.ctrl[act_id] + ctrl
        for act in self.mrl_actuators:
            mju.set_actuator_ctrl(model, data, act, new_ctrl)

    def set_ctrl(
        self, model: mujoco.MjModel, data: mujoco.MjData, ctrl: ArrayLike
    ) -> None:
        assert (
            len(ctrl) == self.n_actuator
        ), f"Control vector should have length {self.n_actuator}, now it is {len(ctrl)}"
        ctrl = np.array(ctrl)
        for i, act in enumerate(self.actuators):
            if act == "j_mrl_fle_A":
                self._set_mrl_ctrl(model, data, ctrl[i])
            else:
                act_id = mju.name2id(self.model, act, "actuator")
                new_ctrl = data.ctrl[act_id] + ctrl[i]
                mju.set_actuator_ctrl(model, data, act, new_ctrl)


class MiaHandOnArm(MiaHand):

    def __init__(
        self, robot_name: str, arm: str, control_type: str = "mocap"
    ) -> None:
        super().__init__(robot_name, control_type)
        self.arm = RobotFactory.create(arm, control_type)
        if self.control_type == "mocap":
            self._excluded_actuators += self.arm.actuators
        else:
            self._excluded_actuators = self.mrl_actuators

    @property
    def actuators(self) -> list[str]:
        all_actuators = mju.get_actuator_names(self.model)

        acts = [
            act for act in all_actuators if act not in self._excluded_actuators
        ]
        acts.append("j_mrl_fle_A")
        return acts

    @property
    def n_actuator(self) -> int:
        return len(self.actuators)


class Ur5Shadow(ShadowHandOnArm):

    def __init__(
        self, name: str = "ur5_shadow", control_type: str = "mocap"
    ) -> None:
        super().__init__(name, "ur5", control_type)


class Ur10Shadow(ShadowHandOnArm):

    def __init__(
        self, name: str = "ur10_shadow", control_type: str = "mocap"
    ) -> None:
        super().__init__(name, "ur10", control_type)


class Ur10eShadow(ShadowHandOnArm):

    def __init__(
        self, name: str = "ur10e_shadow", control_type: str = "mocap"
    ) -> None:
        super().__init__(name, "ur10e", control_type)


class Ur5Mia(MiaHandOnArm):

    def __init__(
        self, name: str = "ur5_mia", control_type: str = "mocap"
    ) -> None:
        super().__init__(name, "ur5", control_type)


class Ur10Mia(MiaHandOnArm):

    def __init__(
        self, name: str = "ur10_mia", control_type: str = "mocap"
    ) -> None:
        super().__init__(name, "ur10", control_type)


class Ur10ftMia(MiaHandOnArm):

    def __init__(
        self, name: str = "ur10ft_mia", control_type: str = "mocap"
    ) -> None:
        super().__init__(name, "ur10ft", control_type)


class Ur10eMia(MiaHandOnArm):

    def __init__(
        self, name: str = "ur10e_mia", control_type: str = "mocap"
    ) -> None:
        super().__init__(name, "ur10e", control_type)


class RobotFactory:

    @staticmethod
    def create(name: str, control_type: str) -> MJRobot:
        if name == "shadow_hand":
            return ShadowHand(name, control_type)
        elif name == "ur5_shadow":
            return Ur5Shadow(name, control_type)
        elif name == "ur10_shadow":
            return Ur10Shadow(name, control_type)
        elif name == "ur10e_shadow":
            return Ur10eShadow(name, control_type)
        elif name == "mia_hand":
            return MiaHand(name, control_type)
        elif name == "ur5_mia":
            return Ur5Mia(name, control_type)
        elif name == "ur10_mia":
            return Ur10Mia(name, control_type)
        elif name == "ur10ft_mia":
            return Ur10ftMia(name, control_type)
        elif name == "ur10e_mia":
            return Ur10eMia(name, control_type)
        # ----- Not used for environment Just need the description ----- #
        elif name == "ur5":
            return Ur5()
        elif name == "ur10":
            return Ur10()
        elif name == "ur10e":
            return Ur10e()
        elif name == "ur10ft":
            return Ur10ft()
        # ----- Not used for environment Just need the description ----- #
        else:
            raise ValueError(f"Robot {name} not found.")
