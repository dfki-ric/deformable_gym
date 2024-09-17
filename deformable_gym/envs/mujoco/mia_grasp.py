from __future__ import annotations

from typing import Dict

from .grasp_env import GraspEnv


class MiaHandGrasp(GraspEnv):
    def __init__(
        self,
        obj_name: str,
        observable_object_pos: bool = True,
        control_type: str = "mocap",
        max_sim_time: float = 2,
        gui: bool = True,
        mocap_cfg: Dict[str, str] | None = None,
        init_frame: str | None = None,
        **kwargs,
    ):
        super().__init__(
            "mia_hand",
            obj_name,
            observable_object_pos,
            control_type,
            max_sim_time,
            gui,
            mocap_cfg,
            init_frame,
            **kwargs,
        )
