import numpy as np
from gym import spaces
from deformable_gym.robots import shadow_hand
from deformable_gym.envs.base_env import BaseBulletEnv, GraspDeformableMixin, FloatingHandMixin
from deformable_gym.objects.bullet_object import ObjectFactory


class ShadowGraspEnv(FloatingHandMixin, GraspDeformableMixin, BaseBulletEnv):
    """Basic grasping environment using only the Shadow dexterous hand.

    The goal is to grasp an object using joint position offsets as actions.

    **State space:**
    Positions of 24 joints:
    WRJ: 2,1; rh_FFJ: 4,3,2,1; rh_MFJ: 4,3,2,1; rh_RFJ: 4,3,2,1;
    rh_LFJ: 5,4,3,2,1; rh_THJ: 5,4,3,2,1.

    **Action space:**  TODO only 20 degrees are controlled actively
    24 joint commands and a flag to indicate termination of an episode.

    :param gui: Show PyBullet GUI (default: True).
    :param real_time: Run simulation in real time (default: False).
    :param object_name: Name of the object to be loaded: 'insole',
        'pillow_small', or 'box' (default: 'insole').
    :param initial_joint_positions: Initial positions of six finger joints
        (default: zeros).
    :param verbose: Print debug output (default: False).
    :param time_delta: Simulation time step. Note that the hand will still be
        controlled at 20 Hz (default: 0.0001).
    :param verbose_dt: Time delta after which the timing module should print
        timing information (default: 0.1).
    """

    HORIZON = 500
    ACCUMULATED_FORCES_THRESHOLD = 200.0  # greater forces end the episode
    DROP_TEST_TIME = 0.5  # time to test grasp stability in the end

    def __init__(
            self, gui=True, real_time=False, object_name="insole",
            initial_joint_positions=(-0.5, -0.0,             # WRJ2, WRJ1
                                     0.3, 0, 0, 0,       # rh_FFJ: 4,3,2,1
                                     0.3, 0, 0, 0,       # rh_MFJ: 4,3,2,1
                                     0.3, 0, 0, 0,       # rh_RFJ: 4,3,2,1
                                     0.3, 0, 0, 0, 0,    # rh_LFJ: 5,4,3,2,1
                                     0.3, 0, 0, 0, 0,    # rh_THJ: 5,4,3,2,1
                                     ),
            verbose=False, time_delta=0.0001,
            verbose_dt=0.1, velocity_control=False):

        self.initial_joint_positions = np.array(initial_joint_positions)
        self.object_name = object_name
        self.time_delta = time_delta
        self.verbose = verbose
        self.velocity_control = velocity_control

        super().__init__(
            gui=gui, real_time=real_time, horizon=self.HORIZON, soft=True,
            load_plane=True, verbose=verbose, time_delta=time_delta,
            verbose_dt=verbose_dt)

        self.hand_world_pose = (0, -0.2, 1, -np.pi / 2, -np.pi, 0)
        self.transforms_initialized = False
        self.robot = self._create_robot()

        self.step_counter = 0

        joint_lower, joint_upper = self.robot.get_joint_limits()
        lower = np.concatenate([joint_lower], axis=0)
        upper = np.concatenate([joint_upper], axis=0)

        self.observation_space = spaces.Box(low=lower, high=upper)
        self.action_space = spaces.Box(
            low=-np.ones(25), high=np.ones(25))

        self.first_step = True
        self.invalid_initial_pose = False

    def _create_robot(self) -> shadow_hand.ShadowHand:
        if self.velocity_control:
            robot = shadow_hand.ShadowHandVelocity(
                verbose=self.verbose, world_pos=self.hand_world_pose[:3],
                world_orn=self.hand_world_pose[3:])
        else:
            robot = shadow_hand.ShadowHandPosition(
                verbose=self.verbose, world_pos=self.hand_world_pose[:3],
                world_orn=self.hand_world_pose[3:])

        self._init_hand_pose(robot)

        robot.set_initial_joint_positions(self.initial_joint_positions)

        self.simulation.add_robot(robot)

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
            state = self.observe_state()
        else:
            self.object_to_grasp.reset()
            state = super().reset()

        self.simulation.camera.reset(self.object_to_grasp.get_pose()[:3], 15,
                                     -105, 0.35)

        self.first_step = True
        self.invalid_initial_pose = False

        return state

    def observe_state(self):
        """Observe state.

        :return: state - Current state of the Mia hand.
        """
        joint_pos = self.robot.get_joint_positions()
        #sensor_readings = self.robot.get_sensor_readings()
        return np.concatenate(
            [joint_pos], axis=0).astype("float64")

    def is_done(self, state, action, next_state):
        """Checks whether the current episode is over or not.

        :param state: Current state of the Mia hand.
        :param action: Action performed by the Mia hand.
        :param next_state: Successor state of the Mia hand.
        :return: done - Is the episode finished?
        """
        high_forces, contact_forces = self._check_forces(
            self.robot, self.ACCUMULATED_FORCES_THRESHOLD, self.verbose)

        self.invalid_initial_pose = self.first_step and contact_forces
        done = (high_forces or self.invalid_initial_pose or
                super(ShadowGraspEnv, self).is_done(state, action, next_state))

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
            for _ in range(int(self.DROP_TEST_TIME / self.time_delta)):
                self.simulation.timing.step()
            return self.object_to_grasp.get_pose()[2]
        else:
            return 0.0
