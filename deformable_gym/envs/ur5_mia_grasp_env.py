import numpy as np
import pytransform3d.transformations as pt
from gymnasium import spaces

from ..envs.base_env import BaseBulletEnv, GraspDeformableMixin
from ..helpers import pybullet_helper as pbh
from ..objects.bullet_object import ObjectFactory
from ..robots import ur5_mia

INITIAL_JOINT_ANGLES = {
    "ur5_shoulder_pan_joint": 2.44388798,
    "ur5_shoulder_lift_joint": -2.01664781,
    "ur5_elbow_joint": 1.72892952,
    "ur5_wrist_1_joint": -0.3965438,
    "ur5_wrist_2_joint": 1.18004689,
    "ur5_wrist_3_joint": 0.18013334,
    "j_index_fle": 0.0,
    "j_little_fle": 0.0,
    "j_mrl_fle": 0.0,
    "j_thumb_opp": 0.0,
    "j_ring_fle": 0.0,
    "j_thumb_fle": 0.0,
}


class UR5MiaGraspEnv(GraspDeformableMixin, BaseBulletEnv):
    """Grasp an insole with UR5 + Mia hand.

    **State space:**
    The state space has 16 dimensions.
    The pose of the end-effector (7 values), finger joint angles (3 values),
    and sensor readings (6 values).
    End-effector poses are defined by position (x, y, z) and scalar-last
    quaternion (qx, qy, qz, qw).
    Joint positions are ordered as follows:
    - (0) j_index_fle
    - (2) j_mrl_fle
    - (5) j_thumb_fle
    Force measurements are ordered as follows:
    - (0) tangential force at middle finger,
    - (1) normal force at index finger,
    - (2) tangential force at index finger,
    - (3) tangential force at thumb,
    - (4) normal force at thumb,
    - (5) normal force at middle finger.

    **Action space:**
    The action space has 10 dimensions.
    End-effector pose (7 values) and joint angles of the hand (3 values).
    End-effector commands are defined by position (x, y, z) and scalar-last
    quaternion (qx, qy, qz, qw).
    Joint commands are ordered as follows:
    - (0) j_index_fle
    - (1) j_mrl_fle
    - (3) j_thumb_fle

    Parameters
    ----------
    :param gui: Show PyBullet GUI.
    :param real_time: Simulate environment in real time.
    :param object_name: Name of the object to be loaded: 'insole',
        'pillow_small', or 'box' (default: 'insole').
    :param verbose: Verbose output.
    :param horizon: Number of steps in simulation.
    :param train: Bool flag indicating if the starting position of the grasped
    object should be sampled from the training or the test set.
    :param thumb_adducted: The joint j_thumb_opp can only take two
        positions (adduction, abduction) and we cannot easily switch between
        them. That's why we have to configure it in advance and cannot control
        it during execution of a policy. When the thumb is adducted, it is
        closer to the palm. When it is abducted, it is further away from the
        palm.
    :param compute_reward: Reward will always be 0 otherwise.
    :param object_scale: Scale factor of the loaded object mesh.
    :param verbose_dt: Time delta after which the timing module should print
        timing information (default: 0.1).
    :param pybullet_options: Options to pass to pybullet.connect().
    """

    robot: ur5_mia.UR5Mia

    object2world = pt.transform_from(R=np.eye(3), p=np.array([-0.7, 0.1, 1.8]))

    def __init__(
        self,
        object_name: str = "insole",
        thumb_adducted: bool = True,
        object_scale: float = 1.0,
        observable_object_pos: bool = False,
        **kwargs,
    ):

        self.velocity_commands = False
        self.object_name = object_name
        self.randomised = False
        self.thumb_adducted = thumb_adducted
        self.object_scale = object_scale

        super().__init__(soft=True, **kwargs)

        self.robot = self._create_robot()
        self._observable_object_pos = observable_object_pos

        limits = pbh.get_limit_array(self.robot.motors.values())

        lower_observations = np.concatenate(
            [
                np.array([-2, -2, 0]),
                -np.ones(4),
                limits[0][6:],
                np.array([-5, -5, -5]),
            ],
            axis=0,
        )

        upper_observations = np.concatenate(
            [
                np.array([2, 2, 2]),
                np.ones(4),
                limits[1][6:],
                np.array([5, 5, 5]),
            ],
            axis=0,
        )

        if self._observable_object_pos:
            lower_observations = np.append(lower_observations, -np.full(3, 2.0))
            upper_observations = np.append(upper_observations, np.full(3, 2.0))

        self.observation_space = spaces.Box(
            low=lower_observations, high=upper_observations
        )

        actuated_finger_ids = np.array([6, 7, 11], dtype=int)

        lower_actions = np.concatenate(
            [
                np.array([-2, -2, 0]),
                -np.ones(4),
                limits[0][actuated_finger_ids],
            ],
            axis=0,
        )

        upper_actions = np.concatenate(
            [np.array([2, 2, 2]), np.ones(4), limits[1][actuated_finger_ids]],
            axis=0,
        )

        self.action_space = spaces.Box(low=lower_actions, high=upper_actions)

    def _create_robot(self):
        task_space_limit = None
        orn_limit = None

        if self.velocity_commands:
            robot = ur5_mia.UR5MiaVelocity(
                pb_client=self.pb_client,
                task_space_limit=task_space_limit,
                end_effector_link="palm",
                verbose=self.verbose,
                orn_limit=orn_limit,
            )
        else:
            robot = ur5_mia.UR5MiaPosition(
                pb_client=self.pb_client,
                task_space_limit=task_space_limit,
                end_effector_link="palm",
                verbose=self.verbose,
                orn_limit=orn_limit,
            )

        self.simulation.add_robot(robot)

        robot.set_initial_joint_positions(INITIAL_JOINT_ANGLES)
        robot.set_thumb_opp(self.thumb_adducted)

        return robot

    def _load_objects(self):
        super()._load_objects()
        (
            self.object_to_grasp,
            self.object_position,
            self.object_orientation,
        ) = ObjectFactory(self.pb_client).create(
            self.object_name,
            object2world=self.object2world,
            scale=self.object_scale,
        )

    def reset(self, seed=None, options=None):
        self.object_to_grasp.reset()
        self.robot.activate_motors()
        return super().reset(seed, options)

    def _get_observation(self):
        joint_pos = self.robot.get_joint_positions(
            self.robot.actuated_real_joints
        )
        ee_pose = self.robot.get_ee_pose()
        sensor_readings = self.robot.get_sensor_readings()

        obs = np.concatenate([ee_pose, joint_pos, sensor_readings], axis=0)

        if self._observable_object_pos:
            obj_pos = self.object_to_grasp.get_pose()[:3]
            obs = np.append(obs, obj_pos)

        return obs

    def calculate_reward(self, state, action, next_state, terminated):
        """
        Calculates the reward by counting how many insole vertices are in the
        target position.
        """
        if terminated:
            self.robot.deactivate_motors()
            # remove insole anchors and simulate steps
            self.object_to_grasp.remove_anchors()
            for _ in range(50):
                if self._deformable_is_exploded():
                    return -1.0
                self.simulation.step_to_trigger("time_step")
            height = self.object_to_grasp.get_pose()[2]
            if height > 0.9:
                return 1.0
            else:
                return -1.0
        else:
            return 0.0
