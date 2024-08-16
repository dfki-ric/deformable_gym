from typing import List, Union

import mujoco
from numpy.typing import NDArray

from ..envs.mujoco.asset_manager import AssetManager
from ..helpers import mj_utils as mju
from ..helpers.mj_utils import Pose


class MJObject:
    """
    A class representing a physical object in a MuJoCo simulation.
    It provides methods to set the object's pose, get the center of mass, and manage equality constraints.

    Attributes:
        name (str): The name of the object, corresponding to its definition in the XML model.
        model (mujoco.MjModel): The MuJoCo model object representing the object in the simulation.
        eq_constraints (List[str]): A list of equality constraints associated with this object.
            These constraints are typically used to enforce specific relationships between bodies,
            such as keeping them at a fixed distance or maintaining an orientation.
        eq_constraints_to_disable (List[Union[str, None]]): A list of equality constraints that are marked to be disabled for this object
            in a specific occasion such as making the object free to move in the simulation for floating objects.
            This list can be customized by subclasses to define which constraints should be ignored or temporarily disabled during the simulation.
    """

    def __init__(self, name: str) -> None:
        self.name = name
        self.model = self._load_model(name)

    @property
    def eq_constraints(self) -> List[str]:
        return mju.get_equality_names(self.model)

    @property
    def eq_constraints_to_disable(self) -> List[Union[str, None]]:
        return []

    def _load_model(self, name: str) -> mujoco.MjModel:
        """
        Load the MuJoCo model for the object.

        This method uses the `AssetManager` to load the XML model file corresponding to the object.

        Args:
            name (str): The name of the object to load.

        Returns:
            mujoco.MjModel: The loaded MuJoCo model for the object.
        """
        return AssetManager().load_asset(name)

    def set_pose(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        pose: Pose,
    ) -> None:
        """
        Set the pose (position and orientation) of the object in the simulation.

        This method updates the position and orientation of the object's body in the simulation
        and recalculates the simulation state.

        Args:
            model (mujoco.MjModel): The MuJoCo model object containing the object's configuration.
            data (mujoco.MjData): The MuJoCo data object containing the current simulation state.
            pose (Pose): A Pose object containing the desired position and orientation for the object.
        """
        model.body(self.name).pos[:] = pose.position
        model.body(self.name).quat[:] = pose.orientation
        mujoco.mj_forward(model, data)

    def get_current_com(self, data: mujoco.MjData) -> NDArray:
        """
        Get the current center of mass (COM) position of the object.

        Args:
            data (mujoco.MjData): The MuJoCo data object containing the current simulation state.

        Returns:
            NDArray: An array representing the position of the object's center of mass.
        """
        return data.body(self.name).ipos


class InsoleFixed(MJObject):

    def __init__(self) -> None:
        super().__init__("insole_fixed")

    @property
    def eq_constraints_to_disable(self) -> List[str]:
        return self.eq_constraints

    def get_current_com(self, data: mujoco.MjData) -> NDArray:
        return data.body(self.name).subtree_com


class ObjectFactory:

    @staticmethod
    def create(name: str) -> MJObject:
        if name == "insole_fixed":
            return InsoleFixed()
        else:
            raise ValueError(f"Object {name} not found")
