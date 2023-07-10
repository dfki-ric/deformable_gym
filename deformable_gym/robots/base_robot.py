import abc
from typing import Tuple, List, Union, Dict, Iterable, Any
import numpy as np
import numpy.typing as npt
import pybullet as pb
from deformable_gym.helpers import pybullet_helper as pbh
from deformable_gym.robots.bullet_utils import draw_limits


class BaseRobot(abc.ABC):
    """Base Robot interface.

    Provides some basic functionality.

    :param urdf_path: Path to URDF file of robot.
    :param world_position: Pose of robot in world coordinates: (x, y, z)
    :param world_orientation: Orientation of the robot in world coordinates (qx, qy, qz, qw).
    :param task_space_limit: Task space limits (position).
    :param fixed_base: Indicates whether the base of the robot is fixed or not.
    """
    def __init__(
            self,
            urdf_path: str,
            world_position: npt.ArrayLike = None,
            world_orientation: npt.ArrayLike = None,
            task_space_limit: Union[npt.ArrayLike, None] = None,
            fixed_base: bool = False):

        self.task_space_limit = task_space_limit
        self.subsystems = {}
        self.fixed_base = fixed_base

        if self.task_space_limit is not None:
            draw_limits(self.task_space_limit)

        if world_position is None:
            self.initial_position = [0, 0, 0]
        else:
            assert len(world_position) == 3
            self.initial_position = world_position

        if world_orientation is None:
            self.initial_orientation = pb.getQuaternionFromEuler((0, 0, 0))
        else:
            assert len(world_orientation) == 3
            self.initial_orientation = pb.getQuaternionFromEuler(world_orientation)

        self._id = self.initialise(urdf_path)

        all_joints = pbh.build_joint_list(self._id, verbose=False)

        # separate fixed joints from controllable motors
        fixed_joints = {
            j.name: j for j in all_joints
            if j.joint_type == pb.JOINT_FIXED
        }

        self.motors = {
            j.name: j for j in all_joints
            if j.joint_type == pb.JOINT_REVOLUTE
        }

        self.sensors = {}

        # collect sensors
        for name, joint in fixed_joints.items():
            if "sensor" in name:
                self.sensors[name] = joint
                pb.enableJointForceTorqueSensor(self._id, joint.joint_idx, 1)

        # set motors initial positions
        for j in self.motors.values():
            j_state = pb.getJointState(self._id, j.joint_idx)
            j.init_pos = j_state[0]
            j.init_vel = j_state[1]

        # create fixed constraint for base if robot base is controllable
        self.base_constraint = \
            pb.createConstraint(self._id,        # parent body id
                                -1,              # parent link id, -1 for base
                                -1,              # child body id, -1 for static frame in world coordinates
                                -1,              # child link id, -1 for base
                                pb.JOINT_FIXED,  # joint type
                                [0, 0, 0],       # joint axis in child frame
                                [0, 0, 0],       # parent frame position
                                [0, 0, 1])       # child frame position

        self.base_pose = pbh.MultibodyPose(
            self.get_id(), self.initial_position, self.initial_orientation)

    def initialise(self, path) -> int:
        """Initialise robot in simulation.

        :return: PyBullet UUID for loaded multi-body.
        """
        return pb.loadURDF(
            path, self.initial_position, self.initial_orientation,
            useFixedBase=self.fixed_base,
            flags=pb.URDF_USE_SELF_COLLISION | pb.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)

    def get_id(self) -> int:
        """Get PyBullet UUID for the robot multi-body.

        :return: PyBullet UUID of robot.
        """
        return self._id

    def set_initial_joint_positions(self, position_dict: Dict[str, float]):
        """Set the initial positions for all the robot's motors.

        :param position_dict: Initial positions of the motors.
        """
        for key in position_dict.keys():
            self.motors[key].init_pos = position_dict[key]

    def get_joint_positions(
            self, keys: Union[Iterable[str], None] = None) -> np.ndarray:
        """Returns the robot's current joint positions.

        :param keys: Names of joints from which positions are requested.
            Defaults to all joints.
        :return: Current positions of the robot's controllable joints.
        """
        if keys is None:
            keys = self.motors.keys()

        return np.array([self.motors[k].get_position() for k in keys])

    def get_joint_velocities(
            self, keys: Union[Iterable[str], None] = None) -> np.ndarray:
        """Returns the robot's current joint velocities.

        :param keys: Names of joints from which velocities are requested.
            Defaults to all joints.
        :return: Current velocities of the robot's controllable joints.
        """
        if keys is None:
            keys = self.motors.keys()
        return np.array([self.motors[k].get_velocity() for k in keys])

    def _get_joint_limits(self, joints: List[str]) -> Tuple[np.ndarray, np.ndarray]:
        """Get array of lower limits and array of upper limits of joints.

        :param joints: Names of joints for which we request limits.
        :return: Array of lower limits and array of upper limits.
        """
        return (np.array([self.motors[joint].low for joint in joints]),
                np.array([self.motors[joint].high for joint in joints]))

