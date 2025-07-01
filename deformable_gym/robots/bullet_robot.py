import abc
from collections.abc import Iterable
from typing import Any, Dict, List, Tuple, Union

import numpy as np
import numpy.typing as npt
import pybullet as pb
from gymnasium.spaces import Box
from pybullet_utils import bullet_client as bc

from ..helpers import pybullet_helper as pbh
from ..objects.bullet_object import Pose
from ..robots.bullet_utils import draw_limits
from ..robots.inverse_kinematics import PyBulletSolver


class BulletRobot(abc.ABC):
    """Base Bullet Robot interface.

    Provides some basic meta functionality.

    :param urdf_path: Path to URDF file of robot.
    :param verbose: Verbosity level.
    :param world_pos: Position of robot in world coordinates: (x, y, z).
    :param world_orn: Orientation of robot in world coordinates represented
    as Euler angles (roll, pitch, yaw; extrinsic concatenation).
    :param control_mode: PyBullet control mode.
    :param task_space_limit: Task space limits (position).
    :param orn_limit: Orientation limits.
    :param base_commands: Control pose of the hand (add 7 components to action
    space).
    """

    def __init__(
        self,
        urdf_path: str,
        pb_client: bc.BulletClient,
        verbose: bool = False,
        world_pos: npt.ArrayLike = None,
        world_orn: npt.ArrayLike = None,
        control_mode: int = pb.POSITION_CONTROL,
        task_space_limit: Union[npt.ArrayLike, None] = None,
        orn_limit: Union[npt.ArrayLike, None] = None,
        base_commands: bool = False,
    ):
        self.path = urdf_path
        self.verbose = verbose
        self.control_mode = control_mode
        self.end_effector = None
        self.inverse_kinematics_solver = None
        self.task_space_limit = task_space_limit
        self.orn_limit = orn_limit
        self.subsystems = {}
        self.base_commands = base_commands
        self.pb_client = pb_client

        if self.task_space_limit is not None and verbose:
            draw_limits(self.task_space_limit, self.pb_client)

        if world_pos is None:
            self.init_pos = [0, 0, 0]
        else:
            assert len(world_pos) == 3
            self.init_pos = world_pos

        if world_orn is None:
            self.init_rot = pb.getQuaternionFromEuler((0, 0, 0))
        else:
            assert len(world_orn) == 4
            self.init_rot = world_orn

        self._id = self.initialise()

        self._joint_name_to_joint_id, self._link_name_to_link_id = (
            pbh.analyze_robot(
                robot=self._id,
                pb_client=self.pb_client,
                verbose=verbose,
            )
        )

        self.all_joints = pbh.build_joint_list(
            self._id, self.pb_client, verbose=self.verbose
        )

        # separate fixed joints from controllable ones
        self.fixed_joints = {
            j.name: j for j in self.all_joints if j.joint_type == pb.JOINT_FIXED
        }

        self.motors = {
            j.name: j
            for j in self.all_joints
            if j.joint_type == pb.JOINT_REVOLUTE
        }

        self.sensors = {}

        # collect sensors
        for name, joint in self.fixed_joints.items():
            if "sensor" in name:
                self.sensors[name] = joint
                self.pb_client.enableJointForceTorqueSensor(
                    self._id, joint.joint_idx, enableSensor=True
                )

        # set the joints initial positions
        for j in self.all_joints:
            j.init_pos = self.pb_client.getJointState(
                self._id,
                j.joint_idx,
            )[0]

        # set initial command
        self.current_command = {k: None for k in self.motors.keys()}

        # create fixed constraint for base if robot base is controllable
        if self.base_commands:
            self.base_constraint = self.pb_client.createConstraint(
                self._id,
                -1,  # parent link id, -1 for base
                -1,  # child body id, -1 for static frame
                -1,  # child link id, -1 for base
                pb.JOINT_FIXED,  # joint type
                [0, 0, 0],  # joint axis in child frame
                [0, 0, 0],  # parent frame position
                [0, 0, 1],  # child frame position
            )

        if verbose:
            print(self.all_joints)

        self.multibody_pose = pbh.MultibodyPose(
            self.get_id(),
            self.init_pos,
            self.init_rot,
            pb_client=self.pb_client,
        )

        # create observation and action spaces
        # TODO: fix to include base actions and sensor observations
        self.action_space = Box(
            np.full(len(self.motors), -1.0), np.full(len(self.motors), 1.0)
        )
        self.observation_space = Box(
            np.full(len(self.motors), -1.0), np.full(len(self.motors), 1.0)
        )

    def initialise(self) -> int:
        """Initialize robot in simulation.

        :return: PyBullet UUID for loaded multi-body.
        """
        return self.pb_client.loadURDF(
            self.path,
            self.init_pos,
            self.init_rot,
            useFixedBase=not self.base_commands,
            flags=pb.URDF_USE_SELF_COLLISION
            | pb.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
        )

    def get_id(self) -> int:
        """Get PyBullet UUID for the robot multi-body.

        :return: PyBullet UUID of robot.
        """
        return self._id

    def set_initial_joint_positions(self, position_dict: dict[str, float]):
        """Set the initial positions for all the robot's motors.

        :param position_dict: Initial positions of the motors.
        """
        for key in position_dict.keys():
            self.motors[key].init_pos = position_dict[key]

    def set_inverse_kinematics_solver(self, inverse_kinematics_solver):
        """Set IK solver.

        :param inverse_kinematics_solver: IK solver.
        """
        self.inverse_kinematics_solver = inverse_kinematics_solver

    def set_endeffector(self, link_id: int) -> None:
        """Set PyBullet id of end-effector link.

        Note that this only has an effect if the PyBulletSolver is used as IK
        solver.

        :param link_id: Link ID.
        """
        self.end_effector = link_id
        if self.inverse_kinematics_solver is None:  # default to PyBullet IK
            self.inverse_kinematics_solver = PyBulletSolver(
                self._id, self.end_effector, self.pb_client
            )

    def get_joint_positions(
        self, keys: Union[Iterable[str], None] = None
    ) -> np.ndarray:
        """Returns the robot's current joint positions.

        :param keys: Names of joints from which positions are requested.
            Defaults to all joints.
        :return: Current positions of the robot's controllable joints.
        """
        if keys is None:
            keys = self.motors.keys()

        return np.array([self.motors[k].get_position() for k in keys])

    def get_joint_velocities(
        self, keys: Union[Iterable[str], None] = None
    ) -> np.ndarray:
        """Returns the robot's current joint velocities.

        :param keys: Names of joints from which velocities are requested.
            Defaults to all joints.
        :return: Current velocities of the robot's controllable joints.
        """
        if keys is None:
            keys = self.motors.keys()
        return np.array([self.motors[k].get_velocity() for k in keys])

    def get_sensor_readings(self) -> np.ndarray:
        """Reads all sensors of the robot.

        :return: Array of sensor readings.
        """
        return np.array(
            [
                el[2][-1]
                for el in self.pb_client.getJointStates(
                    self._id, [j.joint_idx for j in self.sensors.values()]
                )
            ]
        )

    def update_current_command(self, command: dict[str, float]):
        """Updates the current command.

        The current command will be sent to the robot when a time step is
        triggered. This additional storage step is required to allow
        controlling different parts of a single robot with different
        frequencies, e.g., the arm and the hand of a robotic manipulator.

        :param command: New command. A dictionary that maps from motor name to
        command value.
        """
        for key in command.keys():
            self.current_command[key] = command[key]

    def send_current_command(
        self, keys: Union[Iterable[str], None] = None
    ) -> None:
        """Sends the currently stored commands to the selected motors.

        :param keys: The names of the motors to send commands to.
        """
        if self.current_command is not None:
            self.actuate_motors(keys)

    @abc.abstractmethod
    def perform_command(self, command: npt.ArrayLike) -> None:
        """Sends a command to the robot.

        :param command: Robot command.
        """

    @abc.abstractmethod
    def actuate_motors(self, keys: Union[Iterable[str], None] = None) -> None:
        """Takes the provided action and sends the joint controls to the robot.

        :param keys: List of the motor keys to be actuated, if None all motor
        are actuated.
        """

    def compute_ik(
        self,
        joint_positions: np.ndarray,
        pos_command: np.ndarray,
        orn_command: Union[np.ndarray, None] = None,
        velocities: bool = True,
    ) -> np.ndarray:
        """Translates task space command to motor commands.

        :param joint_positions: Current joint angles
        :param pos_command: Positional control signal as (x, y, z).
        :param orn_command: Rotational control signal as (qx, qy, qz, qw).
        :param velocities: Interpret end-effector pose as offsets from current
        pose
        :return: Motor commands.
        """
        assert self.end_effector is not None

        # calculate target position
        if velocities:
            current_pose = self.get_ee_pose()
            current_pos = current_pose[:3]
            current_rot = current_pose[3:]
        else:
            current_pos = np.zeros(3)
            current_rot = np.zeros(4)

        target_pos = current_pos + pos_command

        if self.task_space_limit is not None:
            if self.verbose:
                print(f"Original {target_pos=}")

            target_pos = np.clip(
                target_pos, self.task_space_limit[0], self.task_space_limit[1]
            )

            if self.verbose:
                print(f"Clipped {target_pos=}")

        # calculate target rotation if orientation is provided
        target_orn = None
        if orn_command is not None:
            target_orn = current_rot + orn_command

            if self.orn_limit is not None:
                target_orn_euler = pb.getEulerFromQuaternion(target_orn)
                target_orn_euler_clipped = np.clip(
                    target_orn_euler, self.orn_limit[0], self.orn_limit[1]
                )
                target_orn = pb.getQuaternionFromEuler(target_orn_euler_clipped)

            target_orn /= np.maximum(np.linalg.norm(target_orn), 1e-10)

        if np.isnan(target_pos).any():
            print("FOUND NAN IN TARGET POS!")

        if target_orn is not None and np.isnan(target_orn).any():
            print("FOUND NAN IN TARGET ORN!")

        target = self.inverse_kinematics_solver(
            target_pos, target_orn, joint_positions
        )

        assert np.isfinite(target).all(), f"IK produced NAN:{target=}"

        return target

    def get_ee_pose(self) -> np.ndarray:
        """Returns the current end-effector pose.

        :return: Current end-effector pose as position and scalar-last
        quaternion: (x, y, z, qx, qy, qz, qw).
        """
        if self.end_effector is not None:
            current_pos, current_orn = pbh.link_pose(
                self._id, self.end_effector, self.pb_client
            )
        else:
            current_pos, current_orn = (
                self.pb_client.getBasePositionAndOrientation(self._id)
            )
        return np.concatenate((current_pos, current_orn), axis=0)

    def set_ee_pose(
        self, target_pos: np.ndarray, target_orn: np.ndarray
    ) -> None:
        """Sets the pose of the end-effector instantly.

        No physics simulation steps required.

        :param target_pos: Target position as (x, y, z).
        :param target_orn: Target orientation as (qx, qy, qz, qw).
        """
        joint_pos = self.compute_ik(
            self.get_joint_positions()[:6],
            target_pos,
            target_orn,
            velocities=False,
        )
        position_dict = {
            joint: pos for joint, pos in zip(self.motors.keys(), joint_pos)
        }
        self.set_joint_positions(position_dict)

    def reset(self, keys: Union[Iterable[str], None] = None) -> None:
        """Resets actuators and sensors.

        :param keys: Names of joints to reset. Default is all.
        """
        if self.verbose:
            print("Resetting robot joints!")

        if keys is None:
            keys = self.motors.keys()

        for key in keys:
            self.motors[key].reset()
            self.current_command[key] = self.motors[key].get_position()

    def set_joint_positions(self, position_dict: dict[str, float]) -> None:
        """Sets the joint positions of the robot.

        :param position_dict: Maps joint names to angles.
        """
        for key in position_dict.keys():
            self.motors[key].set_position(position_dict[key])

    def deactivate_motors(self) -> None:
        """Deactivates all motors of the robot."""
        for motor in self.motors.values():
            motor.deactivate()

    def activate_motors(self) -> None:
        """Activates all motors of the robot."""
        for motor in self.motors.values():
            motor.activate()

    def move_base(self, offset: npt.ArrayLike) -> None:
        """Moves the robot base.

        :param offset: Pose offset of the robot given as position and
        scalar-last quaternion: (x, y, z, qx, qy, qz, qw).
        """
        pose = np.hstack(self.multibody_pose.get_pose()) + offset

        target_pos, target_orn = self._enforce_limits(pose)

        target_pos, target_orn = self.multibody_pose.translate_pose(
            target_pos, target_orn
        )
        self.pb_client.changeConstraint(
            self.base_constraint,
            target_pos,
            jointChildFrameOrientation=target_orn,
            maxForce=100000,
        )

    def _enforce_limits(self, pose) -> tuple[Any, Any]:
        assert self.base_commands, (
            "tried to move base, but base commands " "are not enabled"
        )
        if self.task_space_limit is not None:
            target_pos = np.clip(
                pose[:3], self.task_space_limit[0], self.task_space_limit[1]
            )
        else:
            target_pos = pose[:3]
        target_orn = pose[3:]
        # TODO enforce orn_limit
        return target_pos, target_orn

    def reset_base(self, pose: npt.ArrayLike) -> None:
        """Resets the robot base.

        :param pose: Pose of the robot given as position and
        scalar-last quaternion: (x, y, z, qx, qy, qz, qw).
        """
        target_pos, target_orn = self._enforce_limits(pose)

        target_pos, target_orn = self.multibody_pose.set_pose(
            target_pos, target_orn
        )
        self.pb_client.changeConstraint(
            self.base_constraint,
            target_pos,
            jointChildFrameOrientation=target_orn,
            maxForce=100000,
        )

    def _get_link_id(self, link_name: str) -> int:
        """Get PyBullet link ID for link name.

        :param link_name: Name of the link.
        :return: Link ID.
        """
        return self._link_name_to_link_id[link_name]

    def get_joint_limits(
        self, joints: list[str]
    ) -> tuple[np.ndarray, np.ndarray]:
        """Get array of lower limits and array of upper limits of joints.

        :param joints: Names of joints for which we request limits.
        :return: Array of lower limits and array of upper limits.
        """
        return (
            np.array([self.motors[joint].low for joint in joints]),
            np.array([self.motors[joint].high for joint in joints]),
        )


class RobotCommandWrapper:
    """Callback that sends current command to simulation.

    This callback is supposed to be triggered by BulletTiming.

    :param robot: Robot.
    :param joint_names: Names of the actuated joints.
    """

    robot: BulletRobot

    def __init__(self, robot, joint_names):
        self.robot = robot
        self.joint_names = joint_names

    def __call__(self):
        """Send current command to simulation."""
        self.robot.send_current_command(self.joint_names)


class HandMixin:
    debug_visualization: bool
    contact_normals: list[Any]

    def _init_debug_visualizations(self):
        """Initialize debug visualizations."""
        self.robot_pose = Pose(
            np.zeros(3),
            np.array([0.0, 0.0, 0.0, 1.0]),
            pb_client=self.pb_client,
            scale=0.1,
            line_width=1,
        )

        self.object_pose = Pose(
            np.zeros(3),
            np.array([0.0, 0.0, 0.0, 1.0]),
            pb_client=self.pb_client,
            scale=0.1,
            line_width=1,
        )
        self.contact_normals = []

    def get_contact_points(self, object_id: int):
        """Get contact points between robot and given object.

        :param object_id: UUID of object.
        :return: List of contact point information from PyBullet.
        """
        contact_points = self.pb_client.getContactPoints(
            self.get_id(), object_id
        )

        if self.debug_visualization:
            for contact_point_id in self.contact_normals:
                pb.removeUserDebugItem(contact_point_id)

            robot_position, robot_orientation = (
                self.pb_client.getBasePositionAndOrientation(self.get_id())
            )
            self.robot_pose.update(robot_position, robot_orientation)
            object_position, object_orientation = (
                self.pb_client.getBasePositionAndOrientation(object_id)
            )
            self.object_pose.update(object_position, object_orientation)

            self.contact_normals = []
            for contact_point in contact_points:
                position_on_object = contact_point[6]
                contact_normal_on_object = contact_point[7]
                normal_force = contact_point[9]

                object_normal_end = np.array(
                    position_on_object
                ) + normal_force * np.array(contact_normal_on_object)
                contact_normal_line = self.pb_client.addUserDebugLine(
                    position_on_object, object_normal_end, [1, 1, 1]
                )
                self.contact_normals.append(contact_normal_line)
        return contact_points
