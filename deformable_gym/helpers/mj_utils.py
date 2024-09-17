from dataclasses import dataclass, field
from typing import List, Tuple

import mujoco
import numpy as np
from mujoco import MjData, MjModel, mjtJoint
from numpy.typing import ArrayLike, NDArray

# fmt: off
TYPE_NAMES = [
    "body", "xbody", "joint", "dof", "geom", "site", "camera", "light",
    "flex", "mesh", "skin", "hfield", "texture", "material", "pair",
    "exclude", "equality", "tendon", "actuator", "sensor", "numeric",
    "text", "tuple", "key", "plugin",
]
# fmt: on


# -------------------------------- DATA CLASSES --------------------------------#
@dataclass
class Pose:
    position: ArrayLike = field(default_factory=lambda: np.zeros(3))
    orientation: ArrayLike = field(
        default_factory=lambda: np.array([1, 0, 0, 0])
    )

    def __post_init__(self):
        assert len(self.position) == 3, "position should be in shape (3,)."
        assert (
            len(self.orientation) == 3 or len(self.orientation) == 4
        ), "orientation should be in shape (3,) or (4,)."
        self.position = np.array(self.position)
        if len(self.orientation) == 3:
            self.orientation = euler2quat(self.orientation)
        else:
            self.orientation = np.array(self.orientation)


# -------------------------------- LOAD MODEL --------------------------------#
def load_model_from_file(path: str) -> Tuple[MjModel, MjData]:
    model = MjModel.from_xml_path(path)
    data = MjData(model)
    return model, data


def load_model_from_string(xml: str) -> Tuple[MjModel, MjData]:
    model = MjModel.from_xml_string(xml)
    data = MjData(model)
    return model, data


# -------------------------------- BODY UTILS --------------------------------#
def get_body_names(model: MjModel) -> List[str]:

    return [
        model.body(i).name
        for i in range(model.nbody)
        if model.body(i).name != "world"
    ]


def set_body_pose(model: MjModel, data: MjData, name: str, pose: Pose) -> None:
    names = get_body_names(model)
    assert (
        name in names
    ), f"No body named {name} in the model.\n Names available: {names}"
    model.body(name).pos = pose.position
    model.body(name).quat = pose.orientation
    mujoco.mj_forward(model, data)


def remove_body(model: MjModel, data: MjData, name: str) -> None:
    """remove a body by setting its position to (0, 0, -1000).

    Args:
        model (MjModel): mj_Model struct
        data (MjData): mj_Data struct
        name (str): body name
    """
    names = get_body_names(model)
    assert (
        name in names
    ), f"No body named {name} in the model.\n Names available: {names}"

    model.body(name).pos = np.array([0, 0, -1000])
    mujoco.mj_forward(model, data)


def get_body_pose(model: MjModel, data: MjModel, name: str) -> Pose:
    names = get_body_names(model)
    assert (
        name in names
    ), f"No body named {name} in the model.\n Names available: {names}"

    return Pose(data.body(name).xpos, data.body(name).xquat)


def get_body_center_of_mass(
    model: MjModel, data: MjModel, name: str
) -> NDArray:
    names = get_body_names(model)
    assert (
        name in names
    ), f"No body named {name} in the model.\n Names available: {names}"

    return data.body(name).xipos


# -------------------------------- JOINT UTILS --------------------------------#
def get_joint_names(model: MjModel) -> List[str]:
    return [
        model.joint(i).name
        for i in range(model.njnt)
        if model.joint(i).name != ""
    ]


def get_joint_qpos(model: MjModel, data: MjData, *name: str) -> NDArray:
    names = get_joint_names(model)
    assert set(name).issubset(
        names
    ), f"No joint named {name} in the model.\n Names available: {names}"
    return np.concatenate([data.joint(n).qpos for n in name])


def get_joint_qvel(model: MjModel, data: MjData, *name: str) -> NDArray:
    names = set(get_joint_names(model))
    assert set(name).issubset(
        names
    ), f"No joint named {name} in the model.\n Names available: {names}"
    return np.concatenate([data.joint(n).qvel for n in name])


def disable_joint(model: MjModel, data: MjData, *name: str) -> None:
    for n in name:
        joint_type = model.joint(n).type
        if all(
            joint_type == mjtJoint.mjJNT_HINGE
            or joint_type == mjtJoint.mjJNT_SLIDE
        ):
            model.joint(n).range = data.joint(n).qpos
        else:
            raise ValueError(f"Only hinge or slide joint can be disabled.")
    mujoco.mj_forward(model, data)


# -------------------------------- GEOM UTILS --------------------------------#
def get_geom_names(model: MjModel) -> List[str]:
    """Get all first level geom names under worldbody node in the model.

    Args:
        model (MjModel): mj_Model struct

    Returns:
        List[str]: list of all geom names
    """
    return [
        model.geom(i).name
        for i in range(model.ngeom)
        if model.geom(i).name != ""
    ]


def remove_geom(model: MjModel, data: MjData, name: str) -> None:
    """remove a geom by setting its position to (0, 0, -1000).

    Args:
        model (MjModel): mj_Model struct
        data (MjData): mj_Data struct
        name (str): geom name
    """
    names = get_geom_names(model)
    assert (
        name in names
    ), f"No geom named {name} in the model.\n Names available: {names}"

    model.geom(name).pos = np.array([0, 0, -1000])
    mujoco.mj_forward(model, data)


# -------------------------------- SENSOR UTILS --------------------------------#
def get_sensor_names(model: MjModel) -> List[str]:
    return [
        model.sensor(i).name
        for i in range(model.nsensor)
        if model.sensor(i).name != ""
    ]


def get_sensor_data(model: MjModel, data: MjModel, name: str) -> NDArray:
    names = get_sensor_names(model)
    assert (
        name in names
    ), f"No sensor named {name} in the model.\n Names available: {names}"
    return data.sensor(name).data


# -------------------------------- EQUALITY CONSTRAINT UTILS --------------------------------#
def get_equality_names(model: MjModel) -> List[str]:
    return [
        model.equality(i).name
        for i in range(model.neq)
        if model.equality(i).name != ""
    ]


def enable_equality_constraint(
    model: MjModel, data: MjData, *name: str
) -> None:
    names = get_equality_names(model)
    for n in name:
        assert (
            n in names
        ), f"No equality constraint named {n} in the model.\n Names available: {names}"
        id = name2id(model, n, "equality")
        data.eq_active[id] = 1
    mujoco.mj_forward(model, data)


def disable_equality_constraint(
    model: MjModel, data: MjData, *name: str
) -> None:
    names = get_equality_names(model)
    for n in name:
        assert (
            n in names
        ), f"No equality constraint named {n} in the model.\n Names available: {names}"
        id = name2id(model, n, "equality")
        data.eq_active[id] = 0
    mujoco.mj_forward(model, data)


# -------------------------------- ACTUATOR UTILS --------------------------------#
def get_actuator_names(model: MjModel) -> List[str]:
    return [
        model.actuator(i).name
        for i in range(model.nu)
        if model.actuator(i).name != ""
    ]


def set_actuator_ctrl(
    model: MjModel, data: MjData, name: str, ctrl: float
) -> None:
    names = get_actuator_names(model)
    assert (
        name in names
    ), f"No actuator named {name} in the model.\n Names available: {names}"
    id = name2id(model, name, "actuator")
    data.ctrl[id] = ctrl


# -------------------------------- MOCAP UTILS --------------------------------#
def set_mocap_pose(model: MjModel, data: MjData, pose: Pose):
    if model.nmocap != 1:
        raise NotImplementedError("Only one mocap is supported.")

    data.mocap_pos[:] = pose.position
    data.mocap_quat[:] = pose.orientation


# -------------------------------- ROTATION UTILS --------------------------------#
def euler2quat(euler: ArrayLike) -> NDArray:
    assert len(euler) == 3, "input should be in shape (3,)."

    quat = np.zeros(4, dtype=np.float64)
    mujoco.mju_euler2Quat(quat, euler, "xyz")
    return quat


def rotate_quat_by_euler(quat: ArrayLike, euler: ArrayLike) -> NDArray:
    assert len(quat) == 4, "quat should be in shape (4,)."
    assert len(euler) == 3, "euler should be in shape (3,)."

    quat_rot = euler2quat(euler)
    result = np.zeros(4, dtype=np.float64)
    mujoco.mju_mulQuat(result, quat_rot, quat)
    return result


# -------------------------------- OTHER UTILS --------------------------------#
def id2name(model: MjModel, id: int, t: str = "body") -> str:
    assert (
        t in TYPE_NAMES
    ), f"No type name {t} found. Available type names are: {TYPE_NAMES}"

    type_id = mujoco.mju_str2Type(t)
    name = mujoco.mj_id2name(model, type_id, id)
    if name is None:
        raise ValueError(f"No {t}-id {id} found in model")
    return name


def name2id(model: MjModel, name: str, t: str = "body") -> int:
    assert (
        t in TYPE_NAMES
    ), f"No type name {t} found. Available type names are: {TYPE_NAMES}"

    type_id = mujoco.mju_str2Type(t)
    id = mujoco.mj_name2id(model, type_id, name)
    if id == -1:
        raise ValueError(f"No {t}-name {name} found in model")
    return id
