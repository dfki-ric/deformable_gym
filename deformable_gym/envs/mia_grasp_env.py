import numpy as np
import numpy.typing as npt
from gym import spaces
from deformable_gym.robots import mia_hand
from deformable_gym.envs.base_env import BaseBulletEnv, GraspDeformableMixin, FloatingHandMixin
from deformable_gym.objects.bullet_object import ObjectFactory


class MiaGraspEnv(FloatingHandMixin, GraspDeformableMixin, BaseBulletEnv):
    """Basic grasping environment using only the MIA hand.

    The goal is to grasp an object using joint position offsets as actions.

    **State space:**
    The state space has 9 dimensions.
    Position of three joints and values of six force sensors at the fingertips.
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
    The action space has 4 dimensions.
    Joint commands are ordered as follows.
    - (0) j_index_fle
    - (1) j_mrl_fle
    - (3) j_thumb_fle
    At the end there is a flag to indicate that the episode is finished (1)
    or not (0).

    :param gui: Show PyBullet GUI (default: True).
    :param real_time: Run simulation in real time (default: False).
    :param object_name: Name of the object to be loaded: 'insole',
        'pillow_small', or 'box' (default: 'insole').
    :param initial_joint_positions: Initial positions of six finger joints
        (default: zeros).
    :param thumb_adducted: The joint j_thumb_opp can only take two
        positions (adduction, abduction) and we cannot easily switch between
        them. That's why we have to configure it in advance and cannot control
        it during execution of a policy. When the thumb is adducted, it is
        closer to the palm. When it is abducted, it is further away from the
        palm.
    :param verbose: Print debug output (default: False).
    :param time_delta: Simulation time step. Note that the hand will still be
        controlled at 20 Hz (default: 0.0001).
    :param verbose_dt: Time delta after which the timing module should print
        timing information (default: 0.1).
    :param velocity_control: Use velocity control? Default: position control.
    :param abort_on_first_step_with_collision: Abort on first step if the hand
        is in collision with the object.
    """

    HORIZON = 500
    ACCUMULATED_FORCES_THRESHOLD = 200.0  # greater forces end the episode
    DROP_TEST_TIME = 0.5  # time to test grasp stability in the end

    def __init__(
            self,
            gui: bool = True,
            real_time: bool = False,
            object_name: str = "insole",
            initial_joint_positions: npt.ArrayLike = (0, 0, 0, 0, 0, 0),
            thumb_adducted: bool = True,
            verbose: bool = False,
            time_delta: float = 0.0001,
            verbose_dt: float = 0.1,
            velocity_control: bool = False,
            abort_on_first_step_with_collision: bool = False):
        self.initial_joint_positions = np.array(initial_joint_positions)
        self.thumb_adducted = thumb_adducted
        self.object_name = object_name
        self.verbose = verbose
        self.velocity_control = velocity_control
        self.abort_on_first_step_with_collision = abort_on_first_step_with_collision

        super().__init__(
            gui=gui, real_time=real_time, horizon=self.HORIZON, soft=True,
            load_plane=True, verbose=verbose, time_delta=time_delta,
            verbose_dt=verbose_dt)

        self.hand_world_pose = (0, 0, 1, -np.pi / 8, np.pi, 0)
        self.robot = self._create_robot()

        joint_lower, joint_upper = self.robot.get_joint_limits()
        sensor_lower, sensor_upper = self.robot.get_sensor_limits()
        lower = np.concatenate([joint_lower, sensor_lower], axis=0)
        upper = np.concatenate([joint_upper, sensor_upper], axis=0)

        self.observation_space = spaces.Box(low=lower, high=upper)
        self.action_space = spaces.Box(low=-np.ones(4), high=np.ones(4))

        self.first_step = True
        self.invalid_initial_pose = False

    def _create_robot(self) -> mia_hand.MiaHand:
        if self.velocity_control:
            robot = mia_hand.MiaHandVelocity(
                verbose=self.verbose, world_pos=self.hand_world_pose[:3],
                world_orn=self.hand_world_pose[3:],
                debug_visualization=bool(self.verbose))
        else:
            robot = mia_hand.MiaHandPosition(
                verbose=self.verbose, world_pos=self.hand_world_pose[:3],
                world_orn=self.hand_world_pose[3:],
                debug_visualization=bool(self.verbose))

        self.simulation.add_robot(robot)

        self._init_hand_pose(robot)

        robot.set_initial_joint_positions(self.initial_joint_positions)
        robot.set_thumb_opp(self.thumb_adducted)

        return robot

    def _load_objects(self):
        """Load all objects in PyBullet."""
        super()._load_objects()
        self.object_to_grasp, self.object_position, self.object_orientation = \
            ObjectFactory().create(self.object_name)

    def reset(self):
        """Reset episode.

        :return: state - State vector.
        """
        if self.verbose:
            print("Performing reset (april)")

        if self.invalid_initial_pose or self._deformable_is_exploded():
            self._hard_reset()
            state = super().reset()
        else:
            self.object_to_grasp.reset()
            state = super().reset()

        self.simulation.camera.reset(self.object_to_grasp.get_pose()[:3], 0,
                                     50, 0.35)

        self.first_step = True
        self.invalid_initial_pose = False

        return state

    def observe_state(self):
        """Observe state.

        :return: state - Current state of the Mia hand.
        """
        joint_pos = self.robot.get_joint_positions(
            self.robot.actuated_real_joints)
        sensor_readings = self.robot.get_sensor_readings()
        return np.concatenate(
            [joint_pos, sensor_readings], axis=0).astype("float64")

    def is_done(self, state, action, next_state):
        """Checks whether the current episode is over or not.

        :param state: Current state of the Mia hand.
        :param action: Action performed by the Mia hand.
        :param next_state: Successor state of the Mia hand.
        :return: done - Is the episode finished?
        """
        high_forces, contact_forces = self._check_forces(
            self.robot, self.ACCUMULATED_FORCES_THRESHOLD, self.verbose)

        self.invalid_initial_pose = (
                self.abort_on_first_step_with_collision and self.first_step
                and contact_forces)
        done = (high_forces or self.invalid_initial_pose or
                super(MiaGraspEnv, self).is_done(state, action, next_state))

        self.first_step = False

        return done

    def calculate_reward(self, state, action, next_state, done):
        """Calculates the reward given a (state,action,state) tuple.

        :param state: Current state of the Mia hand.
        :param action: Action performed by the Mia hand.
        :param next_state: Successor state of the Mia hand.
        :param done: Is the episode finished?
        """
        if done:
            if self.verbose:
                print("Episode done, calculating final reward...")
            if self.invalid_initial_pose:
                return -1000.0
            # Remove insole anchors and simulate steps.
            self.object_to_grasp.remove_anchors()
            self.simulation.simulate_time(self.DROP_TEST_TIME)
            return self.object_to_grasp.get_pose()[2]
        else:
            return 0.0
