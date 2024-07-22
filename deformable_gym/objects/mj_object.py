import mujoco
import numpy as np
from numpy.typing import ArrayLike, NDArray


class MJObject:

    def __init__(self, model: mujoco.MjModel, name: str) -> None:
        self.model = model
        self.name = name

    def set_position(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        pos: ArrayLike,
    ) -> None:
        assert len(pos) == 3, f"Position should be a 3D vector, now it is {pos}"
        model.body(self.name).pos[:] = pos
        mujoco.mj_forward(model, data)

    def get_com(self, data: mujoco.MjData) -> NDArray:
        return data.body(self.name).ipos


class ObjectFactory:

    @staticmethod
    def create(model: mujoco.MjModel, name: str) -> MJObject:
        if name == "insole_fixed":
            return InsoleFixed(model)
        else:
            raise ValueError(f"Object {name} not found")


class InsoleFixed(MJObject):

    def __init__(self, model: mujoco.MjModel) -> None:
        super().__init__(model, "insole_fixed")

    def get_com(self, data: mujoco.MjData) -> NDArray:
        return data.body(self.name).subtree_com
