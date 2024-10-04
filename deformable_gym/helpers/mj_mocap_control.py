import mujoco
import numpy as np
from mujoco import MjData, MjModel, mjtEq
from numpy.typing import NDArray

from . import mj_utils as mju
from .mj_utils import Pose


class MocapControl:
    """
    A class to control the motion capture (mocap) object in a MuJoCo simulation. It handles attaching
    the mocap body to a simulated object and managing equality constraints such as welds, allowing
    for precise manipulation and control of the object in the environment.

    Parameters:
    -----------
    body_name : str, optional
        The name of the mocap body in the simulation, default is "mocap".
    equality_name : str, optional
        The name of the equality constraint (mjtEq.mjEQ_WELD) used to attach the mocap to another body. Default is "mocap".

    Notes:
    ------
    - This class assumes that only one mocap body is present in the simulation (`model.nmocap == 1`).
    - The control vector `ctrl` should be of shape (6,) with the first 3 elements controlling position
      and the last 3 controlling orientation as Euler angles.
    """

    def __init__(
        self, body_name: str = "mocap", equality_name: str = "mocap"
    ) -> None:
        self.body_name = body_name
        self.equality_name = equality_name

    def eq_is_active(self, model: MjModel, data: MjData) -> bool:
        """
        Checks if the equality constraint for the mocap body is currently active.
        """
        eq_id = mju.name2id(model, self.equality_name, "equality")
        return True if data.eq_active[eq_id] else False

    def enable_eq(self, model: MjModel, data: MjData) -> None:
        """
        Enables the equality constraint (weld) for the mocap body, effectively linking it to another body.
        """
        mju.enable_equality_constraint(model, data, self.equality_name)

    def disable_eq(self, model: MjModel, data: MjData) -> None:
        """
        Disables the equality constraint for the mocap body.

        """
        mju.disable_equality_constraint(model, data, self.equality_name)

    def reset_eq(self, model: MjModel, data: MjData) -> None:
        """
        Resets the equality constraint data, setting the relative position to zero and the quaternion
        to identity. This ensures that the mocap body is aligned with the welded body without any offsets.

        Raises:
        -------
        AssertionError
            If the equality type is not 'weld', which is the only supported type.

        Note:
        -----
        The indication of eq_data is as follows:
        - eq_data[:3]: anchor position
        - eq_data[3:10]: relative position (3D position followed by 4D quaternion orientation)
        - eq_data[10]: torquescale
        since mocap body is attached to the desired body, and no anchor is set,
        we need to set the relative position 0,0,0,1,0,0,0 and anchor position 0,0,0 and torquescale 1 by default
        """

        eq_id = mju.name2id(model, self.equality_name, "equality")
        assert (
            model.equality(eq_id).type == mjtEq.mjEQ_WELD
        ), "Only weld equality is supported."
        model.eq_data[eq_id] = np.array(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        )
        mujoco.mj_forward(model, data)

    def get_weld_body_id(self, model: MjModel) -> int:
        """
        Returns the ID of the body that is welded to the mocap body.
        """
        mocap_body_id = mju.name2id(model, self.body_name)
        body1_id = model.equality(self.equality_name).obj1id
        body2_id = model.equality(self.equality_name).obj2id
        return body1_id if body1_id != mocap_body_id else body2_id

    def attach_mocap2weld_body(self, model: MjModel, data: MjData) -> None:
        """
        Attaches the mocap body to the welded body by aligning their positions and orientations, and resetting the equality constraint.
        """
        weld_body_id = self.get_weld_body_id(model)
        body_xpos = data.body(weld_body_id).xpos
        body_xquat = data.body(weld_body_id).xquat
        mju.set_body_pose(
            model, data, self.body_name, Pose(body_xpos, body_xquat)
        )
        mju.set_mocap_pose(model, data, Pose(body_xpos, body_xquat))
        self.reset_eq(model, data)

    def set_ctrl(self, model: MjModel, data: MjData, ctrl: NDArray) -> None:
        """
        Sets the control inputs for the mocap body. The control vector is expected to have 6 elements:
        the first 3 control the position, and the last 3 control the orientation (Euler angles).
        """
        assert len(ctrl) == 6, "mocap ctrl should be in shape (6,)."
        assert model.nmocap == 1, "Only one mocap body is supported."

        ctrl = np.array(ctrl)
        new_mocap_pos = np.squeeze(data.mocap_pos) + ctrl[:3]
        new_mocap_quat = mju.rotate_quat_by_euler(
            np.squeeze(data.mocap_quat), ctrl[3:]
        )
        mju.set_mocap_pose(model, data, Pose(new_mocap_pos, new_mocap_quat))
