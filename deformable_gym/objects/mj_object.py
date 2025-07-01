from __future__ import annotations

from typing import Dict, List

import mujoco
from mujoco import MjData, MjModel
from numpy.typing import NDArray

from ..envs.mujoco.init_pose import ObjectInitPose
from ..helpers import asset_manager as am
from ..helpers import mj_utils as mju
from ..helpers.mj_utils import Pose


class MJObject:
    """
    A class representing a physical object in a MuJoCo simulation.
    It provides methods to set the object's pose, get the center of mass, and manage equality constraints.

    Attributes:
        name (str): The name of the object, corresponding to its definition in the XML model.
        model (MjModel): The MuJoCo model object representing the object in the simulation.
        eq_constraints (List[str]): A list of equality constraints associated with this object.
            These constraints are typically used to enforce specific relationships between bodies,
            such as keeping them at a fixed distance or maintaining an orientation.
        eq_constraints_to_disable (List[str | None]): A list of equality constraints that are marked to be disabled for this object
            in a specific occasion such as making the object free to move in the simulation for floating objects.
            This list can be customized by subclasses to define which constraints should be ignored or temporarily disabled during the simulation.
    """

    def __init__(self, name: str) -> None:
        self.name = name
        self.model = am.load_asset(name)

    @property
    def init_pose(self) -> dict[str, Pose]:
        return ObjectInitPose.get(self.name)

    @property
    def eq_constraints(self) -> list[str]:
        return mju.get_equality_names(self.model)

    @property
    def eq_constraints_to_disable(self) -> list[str | None]:
        return []

    def set_pose(
        self,
        model: MjModel,
        data: MjData,
        pose: Pose | None,
    ) -> None:
        """
        Set the pose (position and orientation) of the object in the simulation.

        This method updates the position and orientation of the object's body in the simulation
        and recalculates the simulation state.

        Args:
            model (MjModel): The MuJoCo model object containing the object's configuration.
            data (MjData): The MuJoCo data object containing the current simulation state.
            pose (Pose): A Pose object containing the desired position and orientation for the object.
        """
        if pose is not None:
            if pose.position is not None:
                model.body(self.name).pos[:] = pose.position
            if pose.orientation is not None:
                model.body(self.name).quat[:] = pose.orientation
            mujoco.mj_forward(model, data)

    def get_center_of_mass(self, data: MjData) -> NDArray:
        """
        Get the center of mass (COM) position of the object.

        Args:
            data (MjData): The MuJoCo data object containing the current simulation state.

        Returns:
            NDArray: An array representing the position of the object's center of mass.
        """
        return data.body(self.name).ipos


class FixedObject(MJObject):
    def __init__(self, name: str) -> None:
        super().__init__(name)

    @property
    def eq_constraints_to_disable(self) -> list[str]:
        return self.eq_constraints

    def get_center_of_mass(self, data: MjData) -> NDArray:
        return data.body(self.name).subtree_com


class InsoleFixed(FixedObject):

    def __init__(self, name="insole_fixed") -> None:
        super().__init__(name)


class PillowFixed(FixedObject):
    def __init__(self, name="pillow_fixed") -> None:
        super().__init__(name)


class ObjectFactory:

    @staticmethod
    def create(name: str) -> MJObject:
        if name == "insole_fixed":
            return InsoleFixed()
        if name == "pillow_fixed":
            return PillowFixed()
        else:
            raise ValueError(f"Object {name} not found")
