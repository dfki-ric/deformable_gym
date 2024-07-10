from typing import List, Tuple

import mujoco
import numpy as np
from numpy.typing import NDArray

TYPE_NAMES = [
    "body",
    "xbody",
    "joint",
    "dof",
    "geom",
    "site",
    "camera",
    "light",
    "flex",
    "mesh",
    "skin",
    "hfield",
    "texture",
    "material",
    "pair",
    "exclude",
    "equality",
    "tendon",
    "actuator",
    "sensor",
    "numeric",
    "text",
    "tuple",
    "key",
    "plugin",
]


def load_model(path: str) -> Tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)
    return model, data


def get_body_names(model: mujoco.MjModel) -> List[str]:
    """Get all first level body names under worldbody node in the model.

    Args:
        model (mujoco.MjModel): mj_Model struct

    Returns:
        List[str]: list of all body names
    """
    return [
        model.body(i).name
        for i in range(model.nbody)
        if model.body(i).parentid == 0 and model.body(i).name != "world"
    ]


def get_geom_names(model: mujoco.MjModel) -> List[str]:
    """Get all first level geom names under worldbody node in the model.

    Args:
        model (mujoco.MjModel): mj_Model struct

    Returns:
        List[str]: list of all geom names
    """
    return [
        model.geom(i).name
        for i in range(model.ngeom)
        if model.geom(i).name != ""
    ]


def get_sensor_names(model: mujoco.MjModel) -> List[str]:
    return [id2name(model, i, "sensor") for i in range(model.nsensor)]


def remove_body(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> None:
    """remove a body by setting its position to (0, 0, -1000).

    Args:
        model (mujoco.MjModel): mj_Model struct
        data (mujoco.MjData): mj_Data struct
        name (str): body name
    """
    names = get_body_names(model)
    assert (
        name in names
    ), f"No body named {name} in the model.\n Names available: {names}"

    model.body(name).xpos = np.array([0, 0, -1000])
    mujoco.mj_forward(model, data)


def remove_geom(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> None:
    """remove a geom by setting its position to (0, 0, -1000).

    Args:
        model (mujoco.MjModel): mj_Model struct
        data (mujoco.MjData): mj_Data struct
        name (str): geom name
    """
    names = get_geom_names(model)
    assert (
        name in names
    ), f"No geom named {name} in the model.\n Names available: {names}"

    model.geom(name).pos = np.array([0, 0, -1000])
    mujoco.mj_forward(model, data)


def get_body_pos(
    model: mujoco.MjModel, data: mujoco.MjModel, name: str
) -> NDArray:
    names = get_body_names(model)
    assert (
        name in names
    ), f"No body named {name} in the model.\n Names available: {names}"

    return data.body(name).xpos


def get_body_com(
    model: mujoco.MjModel, data: mujoco.MjModel, name: str
) -> NDArray:
    names = get_body_names(model)
    assert (
        name in names
    ), f"No body named {name} in the model.\n Names available: {names}"

    return data.body(name).xipos


def get_sensor_data(
    model: mujoco.MjModel, data: mujoco.MjModel, name: str
) -> NDArray:
    names = get_sensor_names(model)
    assert (
        name in names
    ), f"No sensor named {name} in the model.\n Names available: {names}"
    return data.sensor(name).data


def id2name(model: mujoco.MjModel, id: int, t: str = "body") -> str:
    assert (
        t in TYPE_NAMES
    ), f"No type name {t} found. Available type names are: {TYPE_NAMES}"

    type_id = mujoco.mju_str2Type(t)
    name = mujoco.mj_id2name(model, type_id, id)
    if name is None:
        raise ValueError(f"No {t}-id {id} found in model")
    return name


def name2id(model: mujoco.MjModel, name: str, t: str = "body") -> int:
    assert (
        t in TYPE_NAMES
    ), f"No type name {t} found. Available type names are: {TYPE_NAMES}"

    type_id = mujoco.mju_str2Type(t)
    id = mujoco.mj_name2id(model, type_id, name)
    if id == -1:
        raise ValueError(f"No {t}-name {name} found in model")
    return id


def euler2quat(euler: NDArray) -> NDArray:
    assert euler.shape == (3,), "input should be in shape (3,)."

    quat = np.zeros(4, dtype=np.float64)
    mujoco.mju_euler2Quat(quat, euler, "xyz")
    return quat