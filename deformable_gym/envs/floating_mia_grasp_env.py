import numpy as np
import pybullet as pb
from gymnasium import spaces
from deformable_gym.robots import mia_hand
from deformable_gym.envs.base_env import BaseBulletEnv, GraspDeformableMixin
from deformable_gym.helpers import pybullet_helper as pbh
from deformable_gym.objects.bullet_object import ObjectFactory


class FloatingMiaGraspEnv(GraspDeformableMixin, BaseBulletEnv):
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

    STANDARD_INITIAL_POSE = np.r_[0.03, -0.005, 1.0,
                                   pb.getQuaternionFromEuler([-np.pi/8, np.pi, 0])]

    HARD_INITIAL_POSE = np.r_[0.03, -0.025, 1.0,
                               pb.getQuaternionFromEuler([-np.pi/8, np.pi, 0])]

    _FINGERS_OPEN = {"j_index_fle": 0.0,
                     "j_little_fle": 0.0,
                     "j_mrl_fle": 0.0,
                     "j_ring_fle": 0.0,
                     "j_thumb_fle": 0.0}

    _FINGERS_HALFWAY_CLOSED = {"j_index_fle": 0.5,
                               "j_little_fle": 0.5,
                               "j_mrl_fle": 0.5,
                               "j_ring_fle": 0.5,
                               "j_thumb_fle": 0.3}

    _FINGERS_CLOSED = {"j_index_fle": 0.71,
                       "j_little_fle": 0.71,
                       "j_mrl_fle": 0.71,
                       "j_ring_fle": 0.71,
                       "j_thumb_fle": 0.3}

    _MAX_POS_OFFSET = .0005
    _MAX_ORN_OFFSET = .000005

    def __init__(
            self,
            object_name: str = "insole",
            compute_reward: bool = True,
            object_scale: float = 1.0,
            observable_object_pos: bool = False,
            difficulty_mode: str = "hard",
            **kwargs):

        self.insole = None
        self.velocity_commands = False
        self.object_name = object_name
        self.randomised = False
        self.compute_reward = compute_reward
        self.object_scale = object_scale
        self._observable_object_pos = observable_object_pos

        super().__init__(soft=True, **kwargs)

        self.hand_world_pose = self.STANDARD_INITIAL_POSE
        self.robot = self._create_robot()

        limits = pbh.get_limit_array(self.robot.motors.values())
        self.actuated_finger_ids = np.array([0, 1, 5], dtype=int)

        self.set_difficulty_mode(difficulty_mode)

        lower_observations = np.concatenate([
            np.array([-2, -2, 0]),
            -np.ones(4),
            limits[0][self.actuated_finger_ids],
            -np.full(6, 10)])

        upper_observations = np.concatenate([
            np.array([2, 2, 2]),
            np.ones(4),
            limits[1][self.actuated_finger_ids],
            np.full(6, 10.)])

        if self._observable_object_pos:
            lower_observations = np.append(lower_observations,
                                           -np.full(3, 2.))
            upper_observations = np.append(upper_observations,
                                           np.full(3, 2.))

        self.observation_space = spaces.Box(
            low=lower_observations,
            high=upper_observations,
            dtype=np.float64
        )

        # build the action space
        lower = [-np.full(3, self._MAX_POS_OFFSET),  # max negative base pos offset
                 -np.full(4, self._MAX_ORN_OFFSET),  # max negative base orn offset
                 limits[0][self.actuated_finger_ids]]  # negative joint limits

        upper = [np.full(3, self._MAX_POS_OFFSET),  # max positive base pos offset
                 np.full(4, self._MAX_ORN_OFFSET),  # max positive base orn offset
                 limits[1][self.actuated_finger_ids]]  # positive joint limits

        self.action_space = spaces.Box(low=np.concatenate(lower),
                                       high=np.concatenate(upper),
                                       dtype=np.float64)

    def _create_robot(self):
        orn_limit = None

        if self.velocity_commands:
            robot = mia_hand.MiaHandVelocity(
                world_pos=self.hand_world_pose[:3],
                world_orn=pb.getEulerFromQuaternion(self.hand_world_pose[3:]),
                verbose=self.verbose,
                orn_limit=orn_limit,
                base_commands=True)
        else:
            robot = mia_hand.MiaHandPosition(
                world_pos=self.hand_world_pose[:3],
                world_orn=pb.getEulerFromQuaternion(self.hand_world_pose[3:]),
                verbose=self.verbose,
                orn_limit=orn_limit,
                base_commands=True)

        self.simulation.add_robot(robot)

        return robot

    def _load_objects(self):
        super()._load_objects()
        self.object_to_grasp, self.object_position, self.object_orientation = ObjectFactory().create(self.object_name)

    def set_difficulty_mode(self, mode: str):
        """
        Sets the difficulty of the environment by changing the initial hand
        position. Can be used for curriculum learning.

        :param mode: String representation of the desired difficulty.
        """

        if mode == "hard":
            self.robot.set_initial_joint_positions(self._FINGERS_OPEN)
            self.hand_world_pose = self.HARD_INITIAL_POSE
        elif mode == "medium":
            self.robot.set_initial_joint_positions(self._FINGERS_HALFWAY_CLOSED)
            self.hand_world_pose = self.STANDARD_INITIAL_POSE
        elif mode == "easy":
            self.robot.set_initial_joint_positions(self._FINGERS_CLOSED)
            self.hand_world_pose = self.STANDARD_INITIAL_POSE
        else:
            raise ValueError(f"Received unknown difficulty mode {mode}!")

    def reset(self, seed=None, options=None):

        if options is not None and "initial_pose" in options:
            self.robot.reset_base(options["initial_pose"])
        else:
            self.robot.reset_base(self.hand_world_pose)

        self.object_to_grasp.reset()
        self.robot.activate_motors()

        return super().reset(seed, options)

    def _is_truncated(self, state, action, next_state):
        # check if insole is exploded
        return self._deformable_is_exploded()

    def _get_observation(self):
        joint_pos = self.robot.get_joint_positions(self.robot.actuated_real_joints)
        ee_pose = self.robot.get_ee_pose()
        sensor_readings = self.robot.get_sensor_readings()

        state = np.concatenate([ee_pose, joint_pos, sensor_readings])

        if self._observable_object_pos:
            obj_pos = self.object_to_grasp.get_pose()[:3]
            state = np.append(state, obj_pos)

        return state

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
            if height < 0.9:
                return -1.0
            else:
                return 1.0
        else:
            return 0.0
