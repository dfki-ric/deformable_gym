from __future__ import annotations

import abc
from typing import Any

import gymnasium as gym
import numpy as np
import numpy.typing as npt
import pybullet as pb
import pytransform3d.rotations as pr
from gymnasium import spaces

from ..envs.bullet_simulation import BulletSimulation
from ..helpers.pybullet_helper import MultibodyPose
from ..robots.bullet_robot import BulletRobot


class BaseBulletEnv(gym.Env, abc.ABC):
    """Configures PyBullet for the gym environment.

    :param gui: Show PyBullet GUI.
    :param real_time: Run PyBullet in real time.
    :param horizon: Maximum number of steps in an episode.
    :param soft: Activate soft body simulation.
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
    metadata = {"render_modes": ["human"]}

    def __init__(
        self,
        real_time: bool = False,
        horizon: int = 100,
        soft: bool = False,
        verbose: bool = False,
        time_delta: float = 0.001,
        verbose_dt: float = 10.00,
        pybullet_options: str = "",
        render_mode: str | None = None,
    ):

        self.verbose = verbose
        self.horizon = horizon
        self.render_mode = render_mode

        mode = pb.GUI if render_mode == "human" else pb.DIRECT

        self.simulation = BulletSimulation(
            soft=soft,
            time_delta=time_delta,
            real_time=real_time,
            mode=mode,
            verbose_dt=verbose_dt,
            pybullet_options=pybullet_options,
        )

        self.pb_client = self.simulation.pb_client

        # TODO should we make this configurable? this results in 100 Hz
        self.simulation.timing.add_subsystem("time_step", 100)
        self._load_objects()
        self.step_counter = 0

    @abc.abstractmethod
    def _create_robot(self):
        """Load (or reload) robot to PyBullet simulation."""

    def _load_objects(self):
        """Load objects to PyBullet simulation."""
        self.plane = self.pb_client.loadURDF(
            "plane.urdf", (0, 0, 0), useFixedBase=1
        )

    def _hard_reset(self):
        """Hard reset the PyBullet simulation and reload all objects. May be
        necessary, e.g., if soft-bodies in the environment explode.
        """
        if self.verbose:
            print("Performing hard reset!")

        self.simulation.reset()
        self._load_objects()
        self.robot = self._create_robot()

    def reset(self, seed=None, options=None) -> npt.ArrayLike:
        """Reset the environment to its initial state and returns it.

        :return: Initial state.
        """
        super().reset(seed=seed, options=options)

        if options is not None and "hard_reset" in options:
            self._hard_reset()

        assert isinstance(self.robot, BulletRobot)

        self.robot.reset()
        self.step_counter = 0
        self.simulation.timing.reset()

        # simulate one time step to have correct initial sensor information
        self.simulation.simulate_time(self.simulation.time_delta)

        observation = self._get_observation()
        info = self._get_info()

        if self.verbose:
            print(f"Resetting env: {observation}")

        return observation, info

    def render(self, mode: str = "human"):
        """Render environment.

        This function does not have any effect since you have to specify
        whether we visualize anything already in the constructor. We will only
        check in rendering mode 'human' whether the GUI has been initialized.

        :param mode: Render mode. Only 'human' is allowed.
        """
        if mode == "human":
            assert self.render_mode == "human"
        else:
            raise NotImplementedError(f"Render mode {mode} not supported")

    def _get_observation(self) -> npt.ArrayLike:
        """Returns the current environment state.

        :return: The observation
        """
        return self.robot.get_joint_positions()

    def _get_info(
        self,
        observation: npt.ArrayLike = None,
        action: npt.ArrayLike = None,
        reward: float = None,
        next_observation: npt.ArrayLike = None,
    ) -> dict:
        """Returns the current environment state.

        :return: The observation
        """
        return {}

    def _is_terminated(
        self,
        observation: npt.ArrayLike,
        action: npt.ArrayLike,
        next_observation: npt.ArrayLike,
    ) -> bool:
        """Checks whether the current episode is terminated.

        :param observation: observation before action was taken
        :param action: Action taken in this step
        :param next_observation: Observation after action was taken
        :return: Is the current episode done?
        """
        return self.step_counter >= self.horizon

    def _is_truncated(
        self,
        state: npt.ArrayLike,
        action: npt.ArrayLike,
        next_state: npt.ArrayLike,
    ) -> bool:
        """Checks whether the current episode is truncated.

        :param state: State
        :param action: Action taken in this state
        :param next_state: State after action was taken
        :return: Is the current episode done?
        """
        return False

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
        observation = self._get_observation()

        # execute action
        self.robot.perform_command(action)

        # simulate until next time step
        self.simulation.step_to_trigger("time_step")

        next_observation = self._get_observation()

        # update time step
        self.step_counter += 1

        # check if episode is over
        terminated = self._is_terminated(observation, action, next_observation)
        truncated = self._is_truncated(observation, action, next_observation)

        # calculate the reward
        reward = self.calculate_reward(
            observation, action, next_observation, terminated
        )

        info = self._get_info(observation, action, reward)

        if self.verbose:
            print(
                f"Finished environment step: "
                f"{next_observation=}, "
                f"{reward=}, "
                f"{terminated=}, "
                f"{truncated=}"
            )

        return next_observation, reward, terminated, truncated, info

    def close(self):
        self.simulation.disconnect()

    @abc.abstractmethod
    def calculate_reward(
        self,
        state: npt.ArrayLike,
        action: npt.ArrayLike,
        next_state: npt.ArrayLike,
        terminated: bool,
    ) -> float:
        """Calculate reward.

        :param state: State of the environment.
        :param action: Action that has been executed in the state.
        :param next_state: State after executing the action.
        :param terminated: Is the episode terminated?
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
        return not np.isfinite(self.object_to_grasp.get_pose()).all()

    def _check_forces(self, robot, high_force_threshold, verbose):
        contact_points = robot.get_contact_points(self.object_to_grasp.get_id())
        accumulated_forces = 0.0
        for contact_point in contact_points:
            accumulated_forces += contact_point[9]
        high_forces = accumulated_forces > high_force_threshold
        contact_forces = accumulated_forces > 0

        if high_forces:
            print(f"Accumulated force too high: {accumulated_forces}")
        elif verbose:
            print(f"{accumulated_forces=}")

        return high_forces, contact_forces

    def get_object_pose(self):
        """Get object pose.

        :return: object pose in world coordinates given as position and
                 quaternion: (x, y, z, qw, qx, qy, qz)
        """
        return MultibodyPose.internal_pose_to_external_pose(
            np.hstack((self.object_position, self.object_orientation))
        )


class FloatingHandMixin:
    hand_world_pose: npt.ArrayLike
    multibody_pose: MultibodyPose

    def _init_hand_pose(self, robot: BulletRobot):
        """Initialize hand pose.

        :param robot: Floating hand.
        """
        desired_robot2world_pos = self.hand_world_pose[:3]
        desired_robot2world_orn = pb.getQuaternionFromEuler(
            self.hand_world_pose[3:]
        )
        self.multibody_pose = MultibodyPose(
            robot.get_id(), desired_robot2world_pos, desired_robot2world_orn
        )

    def set_world_pose(self, world_pose):
        """Set pose of the hand.

        :param world_pose: world pose of hand given as position and
                           quaternion: (x, y, z, qw, qx, qy, qz)
        """
        self.hand_world_pose = MultibodyPose.external_pose_to_internal_pose(
            world_pose
        )
        desired_robot2world_pos = self.hand_world_pose[:3]
        desired_robot2world_orn = pb.getQuaternionFromEuler(
            self.hand_world_pose[3:]
        )
        self.multibody_pose.set_pose(
            desired_robot2world_pos, desired_robot2world_orn
        )

    def get_world_pose(self):
        """Get pose of the hand.

        :return: hand pose in world coordinates given as position and
                 quaternion: (x, y, z, qw, qx, qy, qz)
        """
        pos, orn = self.multibody_pose.get_pose()
        return np.r_[pos, pr.quaternion_wxyz_from_xyzw(orn)]
