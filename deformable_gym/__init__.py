from gymnasium.envs.registration import register

__version__ = "0.5.0"


register(
    id="FloatingMiaGraspInsole-v0",
    entry_point="deformable_gym.envs.floating_mia_grasp_env:FloatingMiaGraspEnv",
    disable_env_checker=True,
    kwargs={
        "object_name": "insole_on_conveyor_belt/back",
        "observable_object_pos": True,
    },
)

register(
    id="FloatingMiaGraspPillow-v0",
    entry_point="deformable_gym.envs.floating_mia_grasp_env:FloatingMiaGraspEnv",
    disable_env_checker=True,
    kwargs={
        "object_name": "pillow_small",
        "observable_object_pos": True,
    },
)

register(
    id="FloatingShadowGraspInsole-v0",
    entry_point="deformable_gym.envs.floating_shadow_grasp_env:FloatingShadowGraspEnv",
    disable_env_checker=True,
    kwargs={
        "object_name": "insole_on_conveyor_belt/back",
        "observable_object_pos": True,
    },
)

register(
    id="FloatingShadowGraspPillow-v0",
    entry_point="deformable_gym.envs.floating_shadow_grasp_env:FloatingShadowGraspEnv",
    disable_env_checker=True,
    kwargs={
        "object_name": "pillow_small",
        "observable_object_pos": True,
    },
)

register(
    id="URMiaGraspInsole-v0",
    entry_point="deformable_gym.envs.ur5_mia_grasp_env:UR5MiaGraspEnv",
    disable_env_checker=True,
    kwargs={
        "object_name": "insole_on_conveyor_belt/back",
        "observable_object_pos": True,
    },
)

register(
    id="URMiaGraspPillow-v0",
    entry_point="deformable_gym.envs.ur5_mia_grasp_env:UR5MiaGraspEnv",
    disable_env_checker=True,
    kwargs={
        "object_name": "pillow_small",
        "observable_object_pos": True,
    },
)

register(
    id="URShadowGraspInsole-v0",
    entry_point="deformable_gym.envs.ur10_shadow_grasp_env:UR10ShadowGraspEnv",
    disable_env_checker=True,
    kwargs={
        "object_name": "insole_on_conveyor_belt/back",
        "observable_object_pos": True,
    },
)

register(
    id="URShadowGraspPillow-v0",
    entry_point="deformable_gym.envs.ur10_shadow_grasp_env:UR10ShadowGraspEnv",
    disable_env_checker=True,
    kwargs={
        "object_name": "pillow_small",
        "observable_object_pos": True,
    },
)


def register_mj_grasp_envs(
    version: int = 0,
):
    robot_name2id = {
        "shadow_hand": "FloatingShadow",
        "mia_hand": "FloatingMia",
        "ur5_mia": "UR5Mia",
        "ur10_mia": "UR10Mia",
        "ur10ft_mia": "UR10FTMia",
        "ur10e_mia": "UR10EMia",
        "ur5_shadow": "UR5Shadow",
        "ur10_shadow": "UR10Shadow",
        "ur10e_shadow": "UR10EShadow",
    }
    obj_name2id = {
        "insole_fixed": "Insole",
        "pillow_fixed": "Pillow",
    }

    mia_hand_cam_config = {
        "distance": 1.5,
        "elevation": -40,
        "azimuth": 120,
    }

    shadow_hand_cam_config = {
        "distance": 2,
        "elevation": -40,
        "azimuth": 120,
    }

    hand_on_arm_cam_config = {
        "distance": 3.5,
        "elevation": -40,
        "azimuth": 150,
    }

    for robot_name, robot_id in robot_name2id.items():
        for obj_name, obj_id in obj_name2id.items():
            kwargs = {
                "robot_name": robot_name,
                "obj_name": obj_name,
                "control_type": "joint",
            }
            if robot_name == "shadow_hand":
                cam_config = shadow_hand_cam_config
            elif robot_name == "mia_hand":
                cam_config = mia_hand_cam_config
            else:
                cam_config = hand_on_arm_cam_config
            kwargs.update(default_cam_config=cam_config)
            register(
                id=f"Mj{robot_id}Grasp{obj_id}-v{version}",
                entry_point="deformable_gym.envs.mujoco.grasp_env:GraspEnv",
                disable_env_checker=True,
                kwargs=kwargs,
            )


register_mj_grasp_envs(version=0)
