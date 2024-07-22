from gymnasium.envs.registration import register

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

register(
    id="MjShadowGraspInsole-v0",
    entry_point="deformable_gym.envs.mujoco.shadow_grasp:ShadowHandGrasp",
    disable_env_checker=True,
    kwargs={"obj_name": "insole_fixed"},
)
