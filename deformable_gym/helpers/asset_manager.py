from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from collections.abc import Sequence
from pathlib import Path

import mujoco

ASSETS_DIR = Path(__file__).parents[1] / "assets"
MESH_DIR = os.path.join(ASSETS_DIR, "meshes")
ROBOT_DIR = os.path.join(ASSETS_DIR, "robots", "mjcf")
OBJECT_DIR = os.path.join(ASSETS_DIR, "objects", "mjcf")

ROBOTS = {
    "ur5": os.path.join(ROBOT_DIR, "ur5.xml"),
    "ur10": os.path.join(ROBOT_DIR, "ur10.xml"),
    "ur10ft": os.path.join(ROBOT_DIR, "ur10ft.xml"),
    "ur10e": os.path.join(ROBOT_DIR, "ur10e.xml"),
    "shadow_hand": os.path.join(ROBOT_DIR, "shadow_hand.xml"),
    "mia_hand": os.path.join(ROBOT_DIR, "mia_hand.xml"),
    "ur5_mia": os.path.join(ROBOT_DIR, "mia_hand_on_ur5.xml"),
    "ur10_mia": os.path.join(ROBOT_DIR, "mia_hand_on_ur10.xml"),
    "ur10ft_mia": os.path.join(ROBOT_DIR, "mia_hand_on_ur10_ft.xml"),
    "ur10e_mia": os.path.join(ROBOT_DIR, "mia_hand_on_ur10e.xml"),
    "ur5_shadow": os.path.join(ROBOT_DIR, "shadow_hand_on_ur5.xml"),
    "ur10_shadow": os.path.join(ROBOT_DIR, "shadow_hand_on_ur10.xml"),
    "ur10e_shadow": os.path.join(ROBOT_DIR, "shadow_hand_on_ur10e.xml"),
}
OBJECTS = {
    "insole_fixed": os.path.join(OBJECT_DIR, "insole_fixed.xml"),
    "pillow_fixed": os.path.join(OBJECT_DIR, "pillow_fixed.xml"),
}

SCENE_BASE = os.path.join(ASSETS_DIR, "mj_scene_base.xml")


def load_asset(name: str) -> mujoco.MjModel:
    """
    Loads a asset (robot or object mujoco Model) based on its name.

    Args:
        name (str): The name of the robot or object to be loaded. Must be a key in either
                    ROBOTS or OBJECTS.

    Returns:
        mujoco.MjModel: The loaded MuJoCo model.

    Raises:
        AssertionError: If the specified name is not found in either ROBOTS or OBJECTS.
    """
    assert (
        name in ROBOTS or name in OBJECTS
    ), f"Model {name} not found.\n available: {list(ROBOTS.keys()) + list(OBJECTS.keys())}"
    model_path = ROBOTS.get(name, OBJECTS.get(name))
    model = mujoco.MjModel.from_xml_path(model_path)
    return model


def create_scene(robot_name: str, obj_name: str) -> str:
    """
    Creates an MJCF string representing a MuJoCo scene that includes a robot and an object.

    Args:
        robot_name (str): The name of the robot to include in the scene. Must be a key in ROBOTS.
        obj_name (str): The name of the object to include in the scene. Must be a key in OBJECTS.

    Returns:
        str: A MJCF string representing the combined MuJoCo scene.

    Raises:
        AssertionError: If the specified robot_name is not found in ROBOTS.
        AssertionError: If the specified obj_name is not found in OBJECTS.
    """
    assert (
        robot_name in ROBOTS
    ), f"Robot {robot_name} not found.\n available: {list(ROBOTS.keys())}"
    assert (
        obj_name in OBJECTS
    ), f"Object {obj_name} not found.\n available: {list(OBJECTS.keys())}"

    robot_path = ROBOTS.get(robot_name)
    obj_path = OBJECTS.get(obj_name)

    scene = include_mjcf(obj_path, [robot_path, SCENE_BASE], meshdir=MESH_DIR)
    return scene


def include_mjcf(
    base_path: str,
    include_path: str | Sequence[str],
    *,
    meshdir: str | None = None,
) -> str:
    """
    Generates an XML string for a MuJoCo scene by including additional MJCF files within a base MJCF file.

    Args:
        base_path (str): The file path to the base MJCF file.
        include_path (Union[str, Sequence[str]]): A string or list of strings representing file paths
                                                    to MJCF files to be included in the base file.
        meshdir (Optional[str]): A string representing the path to the directory containing mesh files.
                                    If provided, this path is added to the meshdir attribute of the compiler
                                    element in the MJCF XML.

    Returns:
        str: An XML string representing the combined MJCF file.
    """
    tree = ET.parse(base_path)
    root = tree.getroot()
    if isinstance(include_path, list) or isinstance(include_path, tuple):
        for path in include_path:
            new_elem = ET.Element("include", {"file": path})
            root.insert(0, new_elem)
    else:
        new_elem = ET.Element("include", {"file": include_path})
        root.insert(0, new_elem)
    if meshdir is not None:
        if meshdir[-1] != "/":
            meshdir += "/"
        elems = root.findall("compiler")
        if len(elems) != 0:
            for elem in elems:
                elem.set("meshdir", meshdir)
        else:
            new_elem = ET.Element("compiler", {"meshdir": meshdir})
            root.insert(0, new_elem)
    new_mjcf = ET.tostring(root, encoding="utf-8").decode("utf-8")

    return new_mjcf


def save_mjcf_string(mjcf: str, path: str) -> None:
    with open(path, "w") as f:
        f.write(mjcf)
