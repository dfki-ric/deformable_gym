from gymnasium.envs.registration import register

register(
    id="FloatingMiaGraspInsole-v0",
    entry_point="deformable_gym.envs.floating_mia_grasp_env:FloatingMiaGraspEnv",
    kwargs={
        "object_name": "insole_on_conveyor_belt/back",
        "observable_object_pos": True,
    },
)

register(
    id="FloatingShadowGraspInsole-v0",
    entry_point="deformable_gym.envs.floating_shadow_grasp_env:FloatingShadowGraspEnv",
    kwargs={
        "object_name": "insole_on_conveyor_belt/back",
        "observable_object_pos": True,
    },
)
