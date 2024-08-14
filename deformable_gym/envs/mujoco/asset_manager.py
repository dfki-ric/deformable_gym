import os
import xml.etree.ElementTree as ET
from typing import Optional, Sequence, Union

import mujoco

from ...helpers import mj_utils as mju

ROBOTS = {
    "shadow_hand": "shadow_hand.xml",
    "mia_hand": "mia_hand.xml",
}
OBJECTS = {"insole_fixed": "insole_fixed.xml"}


class AssetManager:

    def __init__(self) -> None:
        self.assets_dir = os.path.join(os.path.dirname(__file__), "assets")
        self.meshdir = os.path.join(self.assets_dir, "meshes")
        self.robots = ROBOTS
        self.objects = OBJECTS

    def load_asset(self, name: str) -> mujoco.MjModel:
        assert (
            name in self.robots or name in self.objects
        ), f"Model {name} not found.\n available: {list(self.robots.keys()) + list(self.objects.keys())}"
        model_path = self.robots.get(name, self.objects.get(name))
        full_path = self._get_full_path(model_path)
        model, _ = mju.load_model_from_file(full_path)
        return model

    def create_scene(self, robot_name: str, obj_name: str) -> str:
        assert (
            robot_name in self.robots
        ), f"Robot {robot_name} not found.\n available: {list(self.robots.keys())}"
        assert (
            obj_name in self.objects
        ), f"Object {obj_name} not found.\n available: {list(self.objects.keys())}"

        floor_path = self._get_full_path("floor.xml")
        robot_path = self._get_full_path(self.robots[robot_name])
        obj_path = self._get_full_path(self.objects[obj_name])

        scene = self.include_mjcf(
            obj_path, [robot_path, floor_path], meshdir=self.meshdir
        )
        return scene

    # def create_scene(self, robot_name: str, *obj_name: str) -> str:
    #     assert (
    #         robot_name in self.robots
    #     ), f"Robot {robot_name} not found.\n available: {list(self.robots.keys())}"

    #     robot_path = self._get_full_path(self.robots[robot_name])

    #     for obj in obj_name:
    #         assert (
    #             obj in self.objects
    #         ), f"Object {obj} not found.\n available: {list(self.objects.keys())}"
    #     obj_path = [self._get_full_path(self.objects[obj]) for obj in obj_name]
    #     scene = self.include_mjcf(robot_path, obj_path, meshdir=self.meshdir)
    #     return scene

    def _get_full_path(self, file: str) -> str:
        return os.path.join(self.assets_dir, file)

    @staticmethod
    def include_mjcf(
        base_path: str,
        include_path: Union[str, Sequence[str]],
        *,
        meshdir: Optional[str] = None,
    ) -> str:
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

    def save_mjcf_string(self, mjcf: str, path: str) -> None:
        with open(path, "w") as f:
            f.write(mjcf)
