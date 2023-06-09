import abc
from typing import Any

import numpy as np
import numpy.typing as npt
import pybullet as pb
import pytransform3d.rotations as pr
from gym.core import Env
from gym import spaces
from deformable_gym.robots.bullet_robot import BulletRobot
from deformable_gym.envs.pybullet_tools import BulletSimulation
from deformable_gym.helpers.pybullet_helper import MultibodyPose


class BaseBulletEnv(Env, abc.ABC):
    """Configures PyBullet for the gym environment.

    :param gui: Show PyBullet GUI.
    :param real_time: Run PyBullet in real time.
    :param horizon: Maximum number of steps in an episode.
    :param soft: Activate soft body simulation.
    :param load_plane: Load plane from URDF.
    :param verbose: Print debug information.
    :param time_delta: Time between PyBullet simulation steps.
    :param verbose_dt: Time after which simulation info should be printed.
    :param pybullet_options: Options to pass to pybullet.connect().
    """
    gui: bool
    verbose: bool
    horizon: int
    simulation: BulletSimulation
    step_counter: int

    robot: BulletRobot
    observation_space: spaces.Box
    action_space: spaces.Box

    def __init__(
            self, gui: bool = True, real_time: bool = False,
            horizon: int = 100, soft: bool = False,
            load_plane: bool = True, verbose: bool = False,
            time_delta: float = 0.0001, verbose_dt: float = 10.00,
            early_episode_termination: bool = True,
            pybullet_options: str = ""):
        self.gui = gui
        self.verbose = verbose
        self.__load_plane = load_plane
        self.horizon = horizon
        self.early_episode_termination = early_episode_termination

        mode = pb.GUI if gui else pb.DIRECT

        self.simulation = BulletSimulation(
            soft=soft, time_delta=time_delta, real_time=real_time, mode=mode,
            verbose_dt=verbose_dt, pybullet_options=pybullet_options)

        # TODO should we make this configurable? this results in 100 Hz
        self.simulation.timing.add_subsystem("time_step", 100)
        self._load_objects()
        self.step_counter = 0

    @abc.abstractmethod
    def _create_robot(self):
        """Load (or reload) robot to PyBullet simulation."""

    def _load_objects(self):
        """Load objects to PyBullet simulation.

        If a plane should be loaded, it will have the position (0, 0, 0).
        """
        if self.__load_plane:
            PLANE_POSITION = (0, 0, 0)
            self.plane = pb.loadURDF(
                "plane.urdf", PLANE_POSITION, useFixedBase=1)

    def _hard_reset(self):
        """Hard reset.

        Fully reset the PyBullet simulation and reload all objects. May be
        necessary, for example, when soft-bodies in the environment explode.
        """
        if self.verbose:
            print("Performing hard reset!")

        self.simulation.reset()
        self._load_objects()
        self.robot = self._create_robot()

    def reset(self) -> np.ndarray:
        """Reset the environment to its initial state and returns it.

        :return: Initial state.
        """

        if self.verbose:
            print("Performing reset (base)")

        assert isinstance(self.robot, BulletRobot)

        self.robot.reset()
        self.step_counter = 0
        self.simulation.timing.reset()

        if self.verbose:
            print(f"Resetting env: {self.observe_state()}")

        return self.observe_state()

    def render(self, mode: str = "human"):
        """Render environment.

        This function does not have any effect since you have to specify
        whether we visualize anything already in the constructor. We will only
        check in rendering mode 'human' whether the GUI has been initialized.

        :param mode: Render mode. Only 'human' is allowed.
        """
        if mode == "human":
            assert self.gui
        else:
            raise NotImplementedError("Render mode '%s' not supported" % mode)

    def observe_state(self) -> np.ndarray:
        """Returns the current environment state.

        :return: The observation
        """
        return self.robot.get_joint_positions()

    def is_done(self, state: np.ndarray, action: np.ndarray,
                next_state: np.ndarray) -> bool:
        """Checks whether the current episode is over or not.

        :param state: State
        :param action: Action taken in this state
        :param next_state: State after action was taken
        :return: Is the current episode done?
        """
        # check for termination action
        if self.early_episode_termination and round(action[-1]) == 1:
            return True

        return self.step_counter >= self.horizon

    def step(self, action: npt.ArrayLike):
        """Take a step in the environment.

        Takes an action (initially target joint positions or joint velocities?)
        and executes it in the (simulated) environment. The function then
        returns the new state, a flag indicating whether the task is
        complete or not, the reward for performing the provided action.
        Optionally, we can also an additional info dict to fully match the
        OpenAI gym environment interface.

        :param action: Action to be taken in this step.
        :return: returns the next (state, reward, done, info) tuple as expected
        for gym environments. Info dict is currently unused.
        """
        # observe current state
        state = self.observe_state()

        # execute action
        if self.early_episode_termination:
            self.robot.perform_command(action[:-1])
        else:
            self.robot.perform_command(action)

        # simulate until next time step
        self.simulation.step_to_trigger("time_step")

        # observe new state
        next_state = self.observe_state()

        # update time step
        self.step_counter += 1

        # check if episode is over
        done = self.is_done(state, action, next_state)

        # calculate the reward
        reward = self.calculate_reward(state, action, next_state, done)

        if self.verbose:
            print(f"Finished environment step: {next_state=}, {reward}, {done=}")

        return next_state, reward, done, {}

    @abc.abstractmethod
    def calculate_reward(
            self, state: np.ndarray, action: np.ndarray,
            next_state: np.ndarray, done: bool):
        """Calculate reward.

        :param state: State of the environment.
        :param action: Action that has been executed in the state.
        :param next_state: State after executing the action.
        :param done: Is the episode done?
        :return: Measured reward.
        """

    def get_robot_id(self):
        """Get object unique ID of robot in PyBullet.

        :return: robot_id - Object unique ID that identifies robot's multi-body.
        """
        return self.robot.get_id()


class GraspDeformableMixin:
    object_to_grasp: Any  # TODO common base class for all objects
    object_position: npt.ArrayLike
    object_orientation: npt.ArrayLike

    def _deformable_is_exploded(self) -> bool:
        """Checks whether a deformable object is exploded.

        May need to be improved in the future. (Potentially allow high rewards
        before explosion is detected). Maybe we can get the object velocity
        instead of the position.

        :return: Whether the deformable is exploded or not.
        """
        pose = self.object_to_grasp.get_pose()

        if np.isfinite(pose).all():
            return np.any(np.abs(pose[:3]) > 3)
        else:
            print("NAN in insole pose -> exploded?!")
            return True

    def _check_forces(self, robot, high_force_threshold, verbose):
        contact_points = robot.get_contact_points(
            self.object_to_grasp.get_id())
        accumulated_forces = 0.0
        for contact_point in contact_points:
            _, _, _, _, _, _, _, _, dist, force, _, _, _, _ = contact_point
            accumulated_forces += force
        high_forces = accumulated_forces > high_force_threshold
        contact_forces = accumulated_forces > 0
        if high_forces:
            print("Accumulated force too high: %g" % accumulated_forces)
        elif verbose:
            print("Accumulated force: %g" % accumulated_forces)
        return high_forces, contact_forces

    def get_object_pose(self):
        """Get object pose.

        :return: object pose in world coordinates given as position and
                 quaternion: (x, y, z, qw, qx, qy, qz)
        """
        return MultibodyPose.internal_pose_to_external_pose(
            np.hstack((self.object_position, self.object_orientation)))


class FloatingHandMixin:
    hand_world_pose: npt.ArrayLike
    multibody_pose: MultibodyPose

    def _init_hand_pose(self, robot: BulletRobot):
        """Initialize hand pose.

        :param robot: Floating hand.
        """
        desired_robot2world_pos = self.hand_world_pose[:3]
        desired_robot2world_orn = pb.getQuaternionFromEuler(self.hand_world_pose[3:])
        self.multibody_pose = MultibodyPose(
            robot.get_id(), desired_robot2world_pos, desired_robot2world_orn)

    def set_world_pose(self, world_pose):
        """Set pose of the hand.

        :param world_pose: world pose of hand given as position and
                           quaternion: (x, y, z, qw, qx, qy, qz)
        """
        self.hand_world_pose = MultibodyPose.external_pose_to_internal_pose(world_pose)
        desired_robot2world_pos = self.hand_world_pose[:3]
        desired_robot2world_orn = pb.getQuaternionFromEuler(self.hand_world_pose[3:])
        self.multibody_pose.set_pose(desired_robot2world_pos, desired_robot2world_orn)

    def get_world_pose(self):
        """Get pose of the hand.

        :return: hand pose in world coordinates given as position and
                 quaternion: (x, y, z, qw, qx, qy, qz)
        """
        pos, orn = self.multibody_pose.get_pose()
        return np.r_[pos, pr.quaternion_wxyz_from_xyzw(orn)]
