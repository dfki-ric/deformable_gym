from typing import Dict

import numpy as np

from ...helpers.mj_utils import Pose


class ObjectInitPose:

    # FIXME: orientation must keep the same as the quat value defined in the mjcf file
    # otherwise, we need to reset the eq constraints

    insole_fixed = {
        "shadow_hand": Pose([0.12, -0.05, 0.456], [0, 0, 0]),
        "ur5_shadow": Pose([1.4, -0.05, 1], [0, 0, 0]),
        "ur10_shadow": Pose([1.55, -0.05, 1], [0, 0, 0]),
        "ur10e_shadow": Pose([1.6, -0.05, 1], [0, 0, 0]),
        "mia_hand": Pose([0.12, -0.05, 0.456], [0, 0, 0]),
        "ur5_mia": Pose([1.4, -0.05, 1], [0, 0, 0]),
        "ur10_mia": Pose([1.55, -0.05, 1], [0, 0, 0]),
        "ur10ft_mia": Pose([1.55, -0.05, 1], [0, 0, 0]),
        "ur10e_mia": Pose([1.6, -0.05, 1], [0, 0, 0]),
    }

    pillow_fixed = {
        "shadow_hand": Pose([0.12, -0.05, 0.456], [0, 0, np.pi / 2]),
        "ur5_shadow": Pose([1.3, -0.05, 1], [0, 0, np.pi / 2]),
        "ur10_shadow": Pose([1.6, -0.05, 1], [0, 0, np.pi / 2]),
        "ur10e_shadow": Pose([1.6, -0.05, 1], [0, 0, np.pi / 2]),
        "mia_hand": Pose([0.12, -0.05, 0.456], [0, 0, np.pi / 2]),
        "ur5_mia": Pose([1.15, -0.05, 1], [0, 0, np.pi / 2]),
        "ur10_mia": Pose([1.5, -0.05, 1], [0, 0, np.pi / 2]),
        "ur10ft_mia": Pose([1.5, -0.05, 1], [0, 0, np.pi / 2]),
        "ur10e_mia": Pose([1.48, -0.05, 1], [0, 0, np.pi / 2]),
    }

    @staticmethod
    def get(name: str) -> dict[str, Pose]:
        if name == "insole_fixed":
            return ObjectInitPose.insole_fixed
        elif name == "pillow_fixed":
            return ObjectInitPose.pillow_fixed
        else:
            return {}


class RobotInitPose:

    shadow_hand = {
        "insole_fixed": Pose([-0.35, 0, 0.49], [0, np.pi / 2, 0]),
    }

    mia_hand = {
        "insole_fixed": Pose([-0.1, 0, 0.49], [0, np.pi, np.pi / 2]),
    }

    @staticmethod
    def get(name: str) -> dict[str, Pose]:
        if name == "shadow_hand":
            return RobotInitPose.shadow_hand
        elif name == "mia_hand":
            return RobotInitPose.mia_hand
        else:
            return {}
