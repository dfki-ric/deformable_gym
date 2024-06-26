import numpy as np
import pybullet as pb
from gymnasium import spaces

from ..envs.grasp_env import GraspEnv
from ..helpers import pybullet_helper as pbh
from ..robots import shadow_hand


class FloatingShadowGraspEnv(GraspEnv):
    """Grasp an insole with a floating Shadow hand.

    **State space:**
    - End-effector pose: (x, y, z, qx, qy, qz, qw)
    - Finger joint angles: 24 values  TODO: order

    **Action space:**
    - End-effector pose: (x, y, z, qx, qy, qz, qw)
    - Finger joint angle: 24 values

    Parameters
    ----------
    :param gui: Show PyBullet GUI.
    :param real_time: Simulate environment in real time.
    :param verbose: Verbose output.
    :param horizon: Number of steps in simulation.
    """

    INITIAL_POSE = np.r_[
        0, -0.5, 1, pb.getQuaternionFromEuler([-np.pi / 2, np.pi, 0])
    ]

    def __init__(
        self,
        object_name: str = "insole",
        object_scale: float = 1.0,
        observable_object_pos: bool = False,
        **kwargs,
    ):
        self.velocity_commands = False

        super().__init__(
            object_name=object_name,
            object_scale=object_scale,
            observable_object_pos=observable_object_pos,
            **kwargs,
        )

        self.hand_world_pose = self.INITIAL_POSE
        self.robot = self._create_robot()

        limits = pbh.get_limit_array(self.robot.motors.values())

        lower_observations = np.concatenate(
            [
                np.array([-2, -2, 0]),
                -np.ones(4),
                limits[0][6:],
                np.array([-10, -10, -10]),
            ],
            axis=0,
        )

        upper_observations = np.concatenate(
            [
                np.array([2, 2, 2]),
                np.ones(4),
                limits[1][6:],
                np.array([10, 10, 10]),
            ],
            axis=0,
        )

        if self._observable_object_pos:
            lower_observations = np.append(lower_observations, -np.full(3, 2.0))
            upper_observations = np.append(upper_observations, np.full(3, 2.0))

        self.observation_space = spaces.Box(
            low=lower_observations, high=upper_observations, dtype=np.float64
        )

        lower_actions = np.concatenate([-np.ones(7) * 0.01, limits[0]], axis=0)

        upper_actions = np.concatenate([np.ones(7) * 0.01, limits[1]], axis=0)

        self.action_space = spaces.Box(
            low=lower_actions, high=upper_actions, dtype=np.float64
        )

    def _create_robot(self):
        task_space_limit = None
        orn_limit = None

        if self.velocity_commands:
            robot = shadow_hand.ShadowHandVelocity(
                pb_client=self.pb_client,
                world_pos=self.hand_world_pose[:3],
                world_orn=self.hand_world_pose[3:],
                task_space_limit=task_space_limit,
                verbose=self.verbose,
                orn_limit=orn_limit,
                base_commands=True,
            )
        else:
            robot = shadow_hand.ShadowHandPosition(
                pb_client=self.pb_client,
                world_pos=self.hand_world_pose[:3],
                world_orn=self.hand_world_pose[3:],
                task_space_limit=task_space_limit,
                verbose=self.verbose,
                orn_limit=orn_limit,
                base_commands=True,
            )

        self.simulation.add_robot(robot)

        return robot

    def _get_observation(self):
        finger_pos = self.robot.get_joint_positions()[6:]
        ee_pose = self.robot.get_ee_pose()

        state = np.concatenate([ee_pose, finger_pos], axis=0)

        if self._observable_object_pos:
            obj_pos = self.object_to_grasp.get_pose()[:3]
            state = np.append(state, obj_pos)

        return state
