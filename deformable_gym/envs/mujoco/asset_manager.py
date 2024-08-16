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
    """
    The AssetManager class manages the loading and combining of MuJoCo models (robots and objects)
    into a single simulation scene. It provides methods to load individual robot or object models,
    create complex scenes, and save the resulting XML configuration.
    """

    def __init__(self) -> None:
        self.assets_dir = os.path.join(os.path.dirname(__file__), "assets")
        self.meshdir = os.path.join(self.assets_dir, "meshes")
        self.robots = ROBOTS
        self.objects = OBJECTS

    def load_asset(self, name: str) -> mujoco.MjModel:
        """
        Loads a asset (robot or object mujoco Model) based on its name.

        Args:
            name (str): The name of the robot or object to be loaded. Must be a key in either
                        self.robots or self.objects.

        Returns:
            mujoco.MjModel: The loaded MuJoCo model.

        Raises:
            AssertionError: If the specified name is not found in either self.robots or self.objects.
        """
        assert (
            name in self.robots or name in self.objects
        ), f"Model {name} not found.\n available: {list(self.robots.keys()) + list(self.objects.keys())}"
        model_path = self.robots.get(name, self.objects.get(name))
        full_path = self._get_full_path(model_path)
        model, _ = mju.load_model_from_file(full_path)
        return model

    def create_scene(self, robot_name: str, obj_name: str) -> str:
        """
        Creates an MJCF string representing a MuJoCo scene that includes a robot and an object.

        Args:
            robot_name (str): The name of the robot to include in the scene. Must be a key in self.robots.
            obj_name (str): The name of the object to include in the scene. Must be a key in self.objects.

        Returns:
            str: A MJCF string representing the combined MuJoCo scene.

        Raises:
            AssertionError: If the specified robot_name is not found in self.robots.
            AssertionError: If the specified obj_name is not found in self.objects.
        """
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
        """
        Generates the full path to a file within the assets directory.

        Args:
            file (str): The file name (with extension) for which to generate the full path.

        Returns:
            str: The full path to the specified file within the assets directory.
        """
        return os.path.join(self.assets_dir, file)

    @staticmethod
    def include_mjcf(
        base_path: str,
        include_path: Union[str, Sequence[str]],
        *,
        meshdir: Optional[str] = None,
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

    def save_mjcf_string(self, mjcf: str, path: str) -> None:
        with open(path, "w") as f:
            f.write(mjcf)
