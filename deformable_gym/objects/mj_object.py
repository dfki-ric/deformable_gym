import mujoco

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
