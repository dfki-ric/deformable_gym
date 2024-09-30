import mujoco
import numpy as np
from mujoco import MjData, MjModel, mjtEq
from numpy.typing import NDArray

from . import mj_utils as mju
from .mj_utils import Pose


class MocapControl:

    def __init__(
        self, body_name: str = "mocap", equality_name: str = "mocap"
    ) -> None:
        self.body_name = body_name
        self.equality_name = equality_name

    def eq_is_active(self, model: MjModel, data: MjData) -> bool:
        eq_id = mju.name2id(model, self.equality_name, "equality")
        return True if data.eq_active[eq_id] else False

    def enable_eq(self, model: MjModel, data: MjData) -> None:
        mju.enable_equality_constraint(model, data, self.equality_name)

    def disable_eq(self, model: MjModel, data: MjData) -> None:
        mju.disable_equality_constraint(model, data, self.equality_name)

    def reset_eq(self, model: MjModel, data: MjData) -> None:
        # the indication of eq_data is as follows:
        # eq_data[:3]: anchor position
        # eq_data[3:10]: relative position (3D position followed by 4D quaternion orientation)
        # eq_data[10]: torquescale
        # since mocap body is attached to the desired body, and no anchor is set,
        # we need to set the relative position 0,0,0,1,0,0,0 and anchor position 0,0,0 and torquescale 1 by default
        eq_id = mju.name2id(model, self.equality_name, "equality")
        assert (
            model.equality(eq_id).type == mjtEq.mjEQ_WELD
        ), "Only weld equality is supported."
        model.eq_data[eq_id] = np.array(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        )
        mujoco.mj_forward(model, data)

    def get_weld_body_id(self, model: MjModel) -> int:
        mocap_body_id = mju.name2id(model, self.body_name)
        body1_id = model.equality(self.equality_name).obj1id
        body2_id = model.equality(self.equality_name).obj2id
        return body1_id if body1_id != mocap_body_id else body2_id

    def attach_mocap2weld_body(self, model: MjModel, data: MjData) -> None:
        weld_body_id = self.get_weld_body_id(model)
        body_xpos = data.body(weld_body_id).xpos
        body_xquat = data.body(weld_body_id).xquat
        mju.set_body_pose(
            model, data, self.body_name, Pose(body_xpos, body_xquat)
        )
        mju.set_mocap_pose(model, data, Pose(body_xpos, body_xquat))
        self.reset_eq(model, data)

    def set_ctrl(self, model: MjModel, data: MjData, ctrl: NDArray) -> None:
        assert len(ctrl) == 6, "mocap ctrl should be in shape (6,)."
        assert model.nmocap == 1, "Only one mocap body is supported."

        ctrl = np.array(ctrl)
        new_mocap_pos = np.squeeze(data.mocap_pos) + ctrl[:3]
        new_mocap_quat = mju.rotate_quat_by_euler(
            np.squeeze(data.mocap_quat), ctrl[3:]
        )
        mju.set_mocap_pose(model, data, Pose(new_mocap_pos, new_mocap_quat))
