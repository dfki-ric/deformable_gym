from __future__ import annotations

from .grasp_env import GraspEnv


class ShadowHandGrasp(GraspEnv):

    def __init__(
        self,
        obj_name: str,
        observable_object_pos: bool = True,
        max_sim_time: float = 1,
        gui: bool = True,
        init_frame: str | None = None,
        **kwargs,
    ):
        super().__init__(
            "shadow_hand",
            obj_name,
            observable_object_pos,
            max_sim_time,
            gui,
            init_frame,
            **kwargs,
        )
