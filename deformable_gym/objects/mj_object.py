import mujoco
from numpy.typing import NDArray

from ..helpers import mj_utils as mju


class MJObject:

    def __init__(self, xml_path) -> None:

        self.model, self.data = mju.load_model(xml_path)

    @property
    def name(self) -> str:
        names = mju.get_body_names(self.model)
        assert (
            len(names) == 1
        ), f"Only one 1st level body is defined in object file, now it is {names}"
        return names[0]

    def get_com(self, data: mujoco.MjData):
        return data.body(self.name).ipos


class ObjectFactory:

    @staticmethod
    def create(name: str, xml_path: str) -> MJObject:
        if name == "insole":
            return Insole(xml_path)
        else:
            raise ValueError(f"Object {name} not found")


class Insole(MJObject):

    def __init__(self, xml_path) -> None:
        super().__init__(xml_path)

    def get_com(self, data: mujoco.MjData) -> NDArray:
        return data.body(self.name).subtree_com
