from typing import List, Tuple

import mujoco
import numpy as np
from numpy.typing import NDArray


def load_model(path: str) -> Tuple[mujoco.MjModel, mujoco.MjData]:
    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)
    return model, data


def id2name(model: mujoco.MjModel, id: int) -> str:
    return model.body(id).name


def name2id(model: mujoco.MjModel, name: str) -> int:
    return model.body(name).id


def euler2quat(euler: NDArray) -> NDArray:
    assert euler.shape == (3,), "input should be in shape (3,)."

    quat = np.zeros(4, dtype=np.float64)
    mujoco.mju_euler2Quat(quat, euler, "xyz")
    return quat


def get_body_names(model: mujoco.MjModel) -> List[str]:

    return [
        model.body(i).name
        for i in range(model.nbody)
        if model.body(i).parentid == 0 and model.body(i).name != "world"
    ]


def get_geom_names(model: mujoco.MjModel) -> List[str]:

    return [
        model.geom(i).name
        for i in range(model.ngeom)
        if model.geom(i).name != ""
    ]


def remove_body(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> None:
    names = get_body_names(model)
    assert (
        name in names
    ), f"No body named {name} in the model.\n Names available: {names}"

    model.body(name).xpos = np.array([0, 0, -100])
    mujoco.mj_forward(model, data)


def remove_geom(model: mujoco.MjModel, data: mujoco.MjData, name: str) -> None:
    names = get_geom_names(model)
    assert (
        name in names
    ), f"No geom named {name} in the model.\n Names available: {names}"

    model.geom(name).pos = np.array([0, 0, -100])
    mujoco.mj_forward(model, data)


def get_body_com(model: mujoco.MjModel, name: str) -> NDArray:
    names = get_body_names(model)
    assert (
        name in names
    ), f"No body named {name} in the model.\n Names available: {names}"

    return model.body(name).ipos
