import numpy as np
import pybullet as pb
from gymnasium import spaces

from ..envs.grasp_env import GraspEnv
from ..helpers import pybullet_helper as pbh
from ..robots import mia_hand


class FloatingMiaGraspEnv(GraspEnv):
    """Grasp an insole with a floating Mia hand.

    **State space:**
    - End-effector pose: (x, y, z, qx, qy, qz, qw)
    - Finger joint angles: thumb_fle, index_fle, mrl_fle
    - Force sensors: 6 values

    **Action space:**
    - End-effector offset: (x, y, z, qx, qy, qz, qw)
    - Finger joint angle: thumb_fle, index_fle, mrl_fle

    Parameters
    ----------
    :param object_name: Name of the object to be loaded: 'insole',
        'pillow_small', or 'box' (default: 'insole').
    :param horizon: Number of steps in simulation.
    :param train: Bool flag indicating if the starting position of the grasped
    :param compute_reward: Compute reward.
    :param object_scale: Scaling factor for object.
    :param task_space_limit: Limits for task space, shape: (2, 3).
    :param gui: Show PyBullet GUI.
    :param real_time: Simulate environment in real time.
    :param verbose: Verbose output.
        object should be sampled from the training or the test set.
    """

    INITIAL_POSE = np.r_[
        0.03, -0.025, 1.0, pb.getQuaternionFromEuler([-np.pi / 8, np.pi, 0])
    ]

    _FINGERS_OPEN = {
        "j_index_fle": 0.0,
        "j_little_fle": 0.0,
        "j_mrl_fle": 0.0,
        "j_ring_fle": 0.0,
        "j_thumb_fle": 0.0,
    }

    _FINGERS_HALFWAY_CLOSED = {
        "j_index_fle": 0.5,
        "j_little_fle": 0.5,
        "j_mrl_fle": 0.5,
        "j_ring_fle": 0.5,
        "j_thumb_fle": 0.3,
    }

    _FINGERS_CLOSED = {
        "j_index_fle": 0.71,
        "j_little_fle": 0.71,
        "j_mrl_fle": 0.71,
        "j_ring_fle": 0.71,
        "j_thumb_fle": 0.3,
    }

    _MAX_POS_OFFSET = 0.0005
    _MAX_ORN_OFFSET = 0.000005

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
        self.actuated_finger_ids = np.array([0, 1, 5], dtype=int)

        lower_observations = np.concatenate(
            [
                np.array([-2, -2, 0]),
                -np.ones(4),
                limits[0][self.actuated_finger_ids],
                -np.full(6, 10),
            ]
        )

        upper_observations = np.concatenate(
            [
                np.array([2, 2, 2]),
                np.ones(4),
                limits[1][self.actuated_finger_ids],
                np.full(6, 10.0),
            ]
        )

        if self._observable_object_pos:
            lower_observations = np.append(lower_observations, -np.full(3, 2.0))
            upper_observations = np.append(upper_observations, np.full(3, 2.0))

        self.observation_space = spaces.Box(
            low=lower_observations, high=upper_observations, dtype=np.float64
        )

        # build the action space
        lower = [
            -np.full(3, self._MAX_POS_OFFSET),  # max negative base pos offset
            -np.full(4, self._MAX_ORN_OFFSET),  # max negative base orn offset
            limits[0][self.actuated_finger_ids],
        ]  # negative joint limits

        upper = [
            np.full(3, self._MAX_POS_OFFSET),  # max positive base pos offset
            np.full(4, self._MAX_ORN_OFFSET),  # max positive base orn offset
            limits[1][self.actuated_finger_ids],
        ]  # positive joint limits

        self.action_space = spaces.Box(
            low=np.concatenate(lower),
            high=np.concatenate(upper),
            dtype=np.float64,
        )

    def _create_robot(self):
        orn_limit = None

        if self.velocity_commands:
            robot = mia_hand.MiaHandVelocity(
                self.pb_client,
                world_pos=self.hand_world_pose[:3],
                world_orn=self.hand_world_pose[3:],
                verbose=self.verbose,
                orn_limit=orn_limit,
                base_commands=True,
            )
        else:
            robot = mia_hand.MiaHandPosition(
                self.pb_client,
                world_pos=self.hand_world_pose[:3],
                world_orn=self.hand_world_pose[3:],
                verbose=self.verbose,
                orn_limit=orn_limit,
                base_commands=True,
            )

        self.simulation.add_robot(robot)

        return robot

    def _get_observation(self):
        joint_pos = self.robot.get_joint_positions(
            self.robot.actuated_real_joints
        )
        ee_pose = self.robot.get_ee_pose()
        sensor_readings = self.robot.get_sensor_readings()

        state = np.concatenate([ee_pose, joint_pos, sensor_readings], axis=0)

        if self._observable_object_pos:
            obj_pos = self.object_to_grasp.get_pose()[:3]
            state = np.append(state, obj_pos)

        return state
