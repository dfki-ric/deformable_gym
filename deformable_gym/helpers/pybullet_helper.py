"""
Helper convenience module to easily transform to and from PyBullet specific
conventions.
"""

import os
import shutil
import sys
import tempfile
from contextlib import contextmanager
from enum import Enum
from importlib.resources import as_file, files

import numpy as np
import numpy.typing as npt
import pybullet as pb
import pytransform3d.rotations as pr
from pybullet_utils import bullet_client as bc


def extract_resources(package, target_dir):
    root = files(package)

    for item in root.rglob("*"):
        if item.is_file():
            relative_path = item.relative_to(root)
            dest_path = os.path.join(target_dir, relative_path)
            os.makedirs(os.path.dirname(dest_path), exist_ok=True)
            with as_file(item) as src_path:
                shutil.copy(src_path, dest_path)


def load_urdf_from_resource(pb_client, resource_name):
    tmp_dir = tempfile.mkdtemp()

    extract_resources("deformable_gym.assets", tmp_dir)
    urdf_relative = os.path.join("robots", "urdf", resource_name)
    urdf_path = os.path.join(tmp_dir, urdf_relative)

    pb_client.setAdditionalSearchPath(tmp_dir)

    return urdf_path


@contextmanager
def stdout_redirected(to=os.devnull):
    fd = sys.stdout.fileno()

    def _redirect_stdout(to_fd):
        os.dup2(to_fd, fd)

    # Duplicate the stdout file descriptor and open it for writing
    old_stdout_fd = os.dup(fd)
    with os.fdopen(old_stdout_fd, "w") as old_stdout:
        with open(to, "w") as file:
            # Redirect stdout to the provided file
            _redirect_stdout(file.fileno())
        try:
            yield
        finally:
            # Redirect stdout back to the old stdout
            _redirect_stdout(old_stdout_fd)


class JointType(Enum):
    revolute = pb.JOINT_REVOLUTE
    prismatic = pb.JOINT_PRISMATIC
    fixed = pb.JOINT_FIXED
    planar = pb.JOINT_PLANAR
    spherical = pb.JOINT_SPHERICAL


class Joint:
    def __init__(
        self,
        name: str,
        joint_idx: int,
        joint_type: JointType,
        pos_idx: int,
        vel_idx: int,
        low: float,
        high: float,
        max_force: float,
        max_vel: float,
        body_id: int,
        pb_client: bc.BulletClient,
    ) -> None:
        self.name = name
        self.joint_idx = joint_idx
        self.joint_type = joint_type
        self.pos_idx = pos_idx
        self.vel_idx = vel_idx
        self.low = low
        self.high = high
        self.max_force = max_force
        self.max_vel = max_vel

        self.init_pos = low
        self.body_id = body_id
        self.activated = True
        self.verbose = False

        self.pb_client = pb_client

    def __repr__(self):
        return (
            f"Joint({', '.join([f'{k}={v}' for k, v in vars(self).items()])})"
        )

    def reset(self) -> None:
        """Resets the joint to its initial position."""
        self.set_position(self.init_pos)

    def set_position(self, position: float) -> None:
        """Sets the joint to target position."""
        self.pb_client.resetJointState(
            self.body_id,
            self.joint_idx,
            targetValue=position,
            targetVelocity=0.0,
        )

    def set_limits(self, low: float, high: float) -> None:
        """Sets the joint limits."""
        self.pb_client.changeDynamics(
            self.body_id,
            self.joint_idx,
            jointLowerLimit=low,
            jointUpperLimit=high,
        )

    def get_position(self) -> float:
        """Gets the current position of the joint."""
        return self.pb_client.getJointState(
            self.body_id,
            self.joint_idx,
        )[0]

    def get_velocity(self) -> float:
        """Gets the current velocity of the joint."""
        return self.pb_client.getJointState(
            self.body_id,
            self.joint_idx,
        )[1]

    def set_target_position(self, position: float) -> None:
        """Sets the target position of the joint."""

        if self.max_vel + 0.1 < self.get_velocity():
            if self.verbose:
                print(
                    f"Joint {self.name} has max velocity of {self.max_vel}"
                    f" but is moving with velocity {self.get_velocity()}"
                )

        if self.activated:
            self.pb_client.setJointMotorControl2(
                self.body_id,
                self.joint_idx,
                controlMode=pb.POSITION_CONTROL,
                targetPosition=position,
                targetVelocity=0.0,
                maxVelocity=self.max_vel,
                force=self.max_force,
            )

        else:
            if self.verbose:
                print(
                    f"""Warning: Trying to control deactivated motor {self.name}."""
                )

    def set_target_velocity(self, velocity: float) -> None:
        """Sets the target position of the joint."""
        if self.activated:
            self.pb_client.setJointMotorControl2(
                self.body_id,
                self.joint_idx,
                controlMode=pb.VELOCITY_CONTROL,
                targetVelocity=velocity,
                maxVelocity=self.max_vel,
                force=self.max_force,
            )
        else:
            if self.verbose:
                print(
                    f"""Warning: Trying to control deactivated motor {self.name}."""
                )

    def deactivate(self):
        """Deactivates the motor."""
        self.set_target_velocity(0.0)
        self.activated = False

    def activate(self):
        """Activates the motor."""
        self.activated = True


def merge_pose(pos, rot):
    """
    Transforms a provided 3D position vector and a 4D quaternion into a 7D pose vector. Also rearranges the quaternion
    to match PyTransform3D conventions.

    :param pos: 3D numpy array containing position of the form [x, y, z]
    :param rot: 4D numpy array containing position and quaternion of the form [qx, qy, qz, qw]

    :return: 7D numpy array containing position and quaternion of the form [x, y, z, qw, qx, qy, qz]
    """
    return np.hstack((pos, [rot[-1]], rot[:-1]))  # xyzw -> wxyz


def build_joint_list(robot, pb_client: bc.BulletClient, verbose: bool = False):
    """
    Builds and returns a dictionary of all joints in the provided robot.

    :param robot: the bullet robot to be examined
    :param verbose:

    :return: list of Joint objects
    """
    n_joints = pb_client.getNumJoints(robot)

    if verbose:
        print(f"Found {n_joints} joints.")

    joint_list = []

    # iterate over all joints
    for joint_idx in range(n_joints):
        (
            _,
            joint_name,
            joint_type,
            pos_idx,
            vel_idx,
            _,
            _,
            _,
            lo,
            hi,
            max_force,
            max_vel,
            *_,
        ) = pb_client.getJointInfo(robot, joint_idx)

        joint_name = joint_name.decode("utf-8")

        joint_list.append(
            Joint(
                joint_name,
                joint_idx,
                joint_type,
                pos_idx,
                vel_idx,
                lo,
                hi,
                max_force,
                max_vel,
                robot,
                pb_client,
            )
        )

    return joint_list


def analyze_robot(
    urdf_path: str = None,
    robot=None,
    pb_client: bc.BulletClient = None,
    verbose=0,
):
    """
    Compute mappings between joint and link names and their indices.
    """

    if urdf_path is not None:
        assert robot is None
        pb_client = pb.connect(pb.DIRECT)
        pb_client.resetSimulation()
        robot = pb_client.loadURDF(urdf_path)

    assert robot is not None

    base_link, robot_name = pb_client.getBodyInfo(robot)

    if verbose:
        print()
        print("=" * 80)
        print(f"Robot name: {robot_name}")
        print(f"Base link: {base_link}")

    n_joints = pb_client.getNumJoints(robot)

    last_link_idx = -1
    link_id_to_link_name = {last_link_idx: base_link}
    joint_name_to_joint_id = {}

    if verbose:
        print(f"Number of joints: {n_joints}")

    for joint_idx in range(n_joints):
        (
            _,
            joint_name,
            joint_type,
            q_index,
            u_index,
            _,
            jd,
            jf,
            lo,
            hi,
            max_force,
            max_vel,
            child_link_name,
            ja,
            parent_pos,
            parent_orient,
            parent_idx,
        ) = pb_client.getJointInfo(robot, joint_idx)

        child_link_name = child_link_name.decode("utf-8")
        joint_name = joint_name.decode("utf-8")

        if child_link_name not in link_id_to_link_name.values():
            last_link_idx += 1
            link_id_to_link_name[last_link_idx] = child_link_name
        assert parent_idx in link_id_to_link_name

        joint_name_to_joint_id[joint_name] = joint_idx

        joint_type = JointType(joint_type).name

        if verbose:
            print(
                f"Joint #{joint_idx}: {joint_name} ({joint_type}), "
                f"child link: {child_link_name}, parent link index: {parent_idx}"
            )
            if joint_type == "fixed":
                continue
            print("=" * 80)
            print(
                f"Index in positional state variables: {q_index}, "
                f"Index in velocity state variables: {u_index}"
            )
            print(
                f"Joint limits: [{lo}, {hi}], max. force: {max_force}, "
                f"max. velocity: {max_vel}"
            )
            print("=" * 80)

    if verbose:
        for link_idx in sorted(link_id_to_link_name.keys()):
            print(f"Link #{link_idx}: {link_id_to_link_name[link_idx]}")

    return joint_name_to_joint_id, {
        v: k for k, v in link_id_to_link_name.items()
    }


def get_limit_array(joint_list):
    return np.asarray([j.low for j in joint_list]), np.asarray(
        [j.high for j in joint_list]
    )


def get_joint_by_name(joint_list, name):
    for j in joint_list:
        if j.name == name:
            return j


def link_pose(
    robot: int, link: int, pb_client: bc.BulletClient
) -> tuple[tuple[float], tuple[float]]:
    """Compute link pose from link state.

    A link state contains the world pose of a link's inertial frame and
    the pose of the link's inertial frame in the link's local reference frame.
    This function will compute the pose of the link's local reference frame.

    pybullet's IK solver needs poses of link frames as input, not of the link's
    inertial frame.

    :param robot: Multi-body UUID.
    :param link: Link ID.
    :return: Tuple of position and scalar-last quaternion.
    """
    (
        link_inertial2world_pos,
        link_inertial2world_orn,
        link_inertial2link_pos,
        link_inertial2link_orn,
    ) = pb_client.getLinkState(robot, link)[:4]
    link2link_inertial_pos, link2link_inertial_orn = pb_client.invertTransform(
        link_inertial2link_pos, link_inertial2link_orn
    )
    link2world_pos, link2world_orn = pb_client.multiplyTransforms(
        link_inertial2world_pos,
        link_inertial2world_orn,
        link2link_inertial_pos,
        link2link_inertial_orn,
    )
    return link2world_pos, link2world_orn


def start_recording(path: str, pb_client: bc.BulletClient):
    # TODO: double check that recording is not already in progress
    logging_id = pb_client.startStateLogging(pb.STATE_LOGGING_VIDEO_MP4, path)
    return logging_id


def stop_recording(logging_id: int, pb_client: bc.BulletClient):
    pb_client.stopStateLogging(logging_id)


class MultibodyPose:
    """A facade that correctly places multibodies at their link frames.

    The functions pybullet.resetBasePositionAndOrientation and
    pybullet.getBasePositionAndOrientation work with the inertia frame of
    the root link. It is not directly possible to set the pose of the
    link frame of the root link.

    Parameters
    ----------
    uuid : int
        UUID of the multibody.

    initial_pos : array-like, shape (3,)
        Initial position when the URDF was loaded.

    initial_orn : array-like, shape (4,)
        Initial orientation when the URDF was loaded. Provided as scalar-last
        quaternion.

    pb_client_id : int, optional (default: 0)
        Physics client ID.
    """

    def __init__(
        self, uuid, initial_pos, initial_orn, pb_client: bc.BulletClient
    ):
        self.uuid = uuid
        self.pb_client = pb_client

        inertia_pos, inertia_orn = self.pb_client.getBasePositionAndOrientation(
            self.uuid
        )
        inv_pos, inv_orn = pb.invertTransform(inertia_pos, inertia_orn)
        self.inertia_offset_pos, self.inertia_offset_orn = (
            self.pb_client.invertTransform(
                *self.pb_client.multiplyTransforms(
                    inv_pos, inv_orn, initial_pos, initial_orn
                )
            )
        )

    def set_pose(self, pos, orn):
        """Set pose of the root link frame with respect to world frame.

        Parameters
        ----------
        pos : array-like, shape (3,)
            Position.

        orn : array-like, shape (4,)
            Orientation. Provided as scalar-last quaternion.

        Returns
        -------
        pos : array-like, shape (3,)
            Inertial frame position.

        orn : array-like, shape (4,)
            Inertial frame orientation. Provided as scalar-last quaternion.
        """
        new_pos, new_orn = self.translate_pose(pos, orn)
        self.pb_client.resetBasePositionAndOrientation(
            self.uuid, new_pos, new_orn
        )
        return new_pos, new_orn

    def translate_pose(self, pos, orn):
        """Translate pose of the root link frame to pose of the root inertial frame.

        Parameters
        ----------
        pos : array-like, shape (3,)
            Position.

        orn : array-like, shape (4,)
            Orientation. Provided as scalar-last quaternion.

        Returns
        -------
        pos : array-like, shape (3,)
            Inertial frame position.

        orn : array-like, shape (4,)
            Inertial frame orientation. Provided as scalar-last quaternion.
        """
        return self.pb_client.multiplyTransforms(
            pos, orn, self.inertia_offset_pos, self.inertia_offset_orn
        )

    def get_pose(self):
        """Get pose of the root link frame with respect to world frame.

        Returns
        -------
        pos : array-like, shape (3,)
            Position.

        orn : array-like, shape (4,)
            Orientation. Provided as scalar-last quaternion.
        """
        measured_pos, measured_orn = (
            self.pb_client.getBasePositionAndOrientation(self.uuid)
        )
        return self.pb_client.multiplyTransforms(
            measured_pos,
            measured_orn,
            *self.pb_client.invertTransform(
                self.inertia_offset_pos, self.inertia_offset_orn
            ),
        )

    @staticmethod
    def external_pose_to_internal_pose(pose: npt.ArrayLike) -> np.ndarray:
        position = pose[:3]
        orientation_qwxyz = pose[3:]
        orientation_qwxyz = pr.norm_vector(orientation_qwxyz)
        R = pr.matrix_from_quaternion(orientation_qwxyz)
        orientation = pr.extrinsic_euler_xyz_from_active_matrix(R)
        return np.hstack((position, orientation))

    @staticmethod
    def internal_pose_to_external_pose(pose: npt.ArrayLike) -> np.ndarray:
        position = pose[:3]
        orientation = pose[3:]
        R = pr.active_matrix_from_extrinsic_roll_pitch_yaw(orientation)
        orientation_qwxyz = pr.quaternion_from_matrix(R)
        return np.hstack((position, orientation_qwxyz))
