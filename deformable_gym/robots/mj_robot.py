from typing import Tuple

import mujoco
import numpy as np
from numpy.typing import NDArray

from ..helpers import mj_utils as mju


class MJRobot:

    def __init__(self, model_path: str, init_frame: str = None):

        self.model, self.data = mju.load_model(model_path)
        self.init_frame = init_frame
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

    def set_state(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        qpos: NDArray,
        qvel: NDArray,
    ):
        assert qpos.shape == (
            self.nq,
        ), f"robot has {self.nq} coorinates. get {qpos.shape} qpos."
        assert qvel.shape == (
            self.dof,
        ), f"robot has {self.dof} dof. get {qvel.shape} qvel"

        data.qpos[: self.nq] = qpos
        data.qvel[: self.dof] = qvel
        mujoco.mj_forward(model, data)

    def load_keyframe(
        self, model: mujoco.MjModel, data: mujoco.MjData, frame_name: str
    ):
        frame = self.model.keyframe(frame_name)
        qpos = frame.qpos.copy()
        qvel = frame.qvel.copy()
        self.set_state(model, data, qpos, qvel)

    def reset(self, model: mujoco.MjModel, data: mujoco.MjData):
        if self.init_frame is not None:
            self.load_keyframe(model, data, self.init_frame)
