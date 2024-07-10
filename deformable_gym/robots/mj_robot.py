from typing import Tuple

import mujoco
import numpy as np
from numpy.typing import NDArray

from ..helpers import mj_utils as mju


class MJRobot:

    def __init__(self, model_path: str, init_frame: str = None):

        self.model, self.data = mju.load_model(model_path)
        self.nq = self.model.nq
        self.dof = self.model.nv
        self.nact = self.model.nu
        self.ctrl_range = self.model.actuator_ctrlrange.copy()

    @property
    def name(self) -> str:
        names = mju.get_body_names(self.model)
        assert (
            len(names) == 1
        ), f"Only one 1st level body is defined in object file, now it is {names}"
        return names[0]

    def get_qpos(self, data: mujoco.MjData) -> NDArray:
        return data.qpos[: self.nq].copy()

    def get_qvel(self, data: mujoco.MjData) -> NDArray:
        return data.qvel[: self.dof].copy()
