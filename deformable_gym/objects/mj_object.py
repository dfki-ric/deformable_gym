from typing import List, Union

import mujoco
from numpy.typing import NDArray

from ..envs.mujoco.asset_manager import AssetManager
from ..helpers import mj_utils as mju
from ..helpers.mj_utils import Pose


class MJObject:

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
        manager = AssetManager()
        return manager.load_asset(name)

    def set_pose(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        pose: Pose,
    ) -> None:
        model.body(self.name).pos[:] = pose.position
        model.body(self.name).quat[:] = pose.orientation
        mujoco.mj_forward(model, data)

    def get_current_com(self, data: mujoco.MjData) -> NDArray:
        return data.body(self.name).ipos

    def disable_eq_constraint(
        self, model: mujoco.MjModel, data: mujoco.MjData, *name: str
    ) -> None:
        if len(name) == 0:
            name = self.eq_constraints_to_disable
        mju.disable_equality_constraint(model, data, *name)


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
