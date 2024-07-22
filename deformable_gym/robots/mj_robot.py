import mujoco
from numpy.typing import ArrayLike, NDArray


class MJRobot:

    def __init__(self, model: mujoco.MjModel, name: str) -> None:

        self.model = model
        self.name = name
        self.nq = self.model.nq
        self.dof = self.model.nv
        self.nact = self.model.nu
        self.ctrl_range = self.model.actuator_ctrlrange.copy()

    def get_qpos(self, data: mujoco.MjData) -> NDArray:
        return data.qpos[: self.nq].copy()

    def get_qvel(self, data: mujoco.MjData) -> NDArray:
        return data.qvel[: self.dof].copy()

    def set_position(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        pos: ArrayLike,
    ) -> None:
        assert len(pos) == 3, f"Position should be a 3D vector, now it is {pos}"
        model.body(self.name).pos[:] = pos
        mujoco.mj_forward(model, data)
