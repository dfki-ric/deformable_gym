from typing import Union
import pybullet as pb
import numpy as np
import numpy.typing as npt
import pytransform3d.rotations as pr
import pytransform3d.transformations as pt
from .mia_grasp_env import MiaGraspEnv
from .floating_mia_grasp_env import FloatingMiaGraspEnv
from grasp_metrics.utils.pybullet_interface import (
    collect_contact_information, compute_inertia)
from grasp_metrics.geometry import fit_plane_from_point_set, point_to_line_segment
from grasp_metrics.quasi_static_point_based import ferrari_canny
from grasp_metrics.geometry_based import (
    distance_com_centroid, distance_grasp_point_centroid, grasp_polygon_shape,
    area_of_grasp_polygon, orthogonality, distance_grasp_point_finger_tips)
from grasp_metrics.hands import MiaHand


class MiaGraspEnvWithMetrics(MiaGraspEnv):
    """Grasping environment using only the MIA hand that uses grasp metrics.

    The goal is to grasp an object using joint position offsets as actions.

    **State space:**
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
    Joint position offset for all four controllable degrees of freedom.
    Commands are ordered as follows:
    - (0) j_index_fle
    - (1) j_mrl_fle
    - (3) j_thumb_fle

    :param gui: Show PyBullet GUI (default: True).
    :param real_time: Run simulation in real time (default: False).
    :param object_name: Name of the object to be loaded: 'insole',
        'pillow_small', or 'box' (default: 'insole').
    :param initial_joint_positions: Initial positions of six finger joints
        (default: zeros).
    :param grasp_point: Grasp point.
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
    """
    def __init__(
            self,
            gui: bool = True,
            real_time: bool = False,
            object_name: str = "insole",
            initial_joint_positions: npt.ArrayLike = (0, 0, 0, 0, 0, 0),
            grasp_point: Union[npt.ArrayLike, None] = None,
            thumb_adducted: bool = False,
            verbose: bool = False,
            time_delta: float = 0.0001,
            verbose_dt: float = 0.1,
            velocity_control: bool = False,
            abort_on_first_step_with_collision: bool = False):
        super(MiaGraspEnvWithMetrics, self).__init__(
            gui, real_time, object_name, initial_joint_positions,
            thumb_adducted, verbose, time_delta, verbose_dt, velocity_control,
            abort_on_first_step_with_collision)
        self.grasp_point = grasp_point

    def calculate_reward(self, state, action, next_state, done):
        """Calculates the reward given a (state,action,state) tuple.

        :param state: Current state of the Mia hand.
        :param action: Action performed by the Mia hand.
        :param next_state: Successor state of the Mia hand.
        :param done: Is the episode finished?
        """
        if done:
            return _final_reward_with_metrics(self, self.DROP_TEST_TIME)
        else:
            return 0.0


class FloatingMiaGraspEnvWithMetrics(FloatingMiaGraspEnv):
    """Grasp an insole with a floating Mia hand.

    **State space:**
    - End-effector pose: (x, y, z, qx, qy, qz, qw)
    - Finger joint angles: thumb_fle, index_fle, mrl_fle
    - Force sensors: 6 values

    **Action space:**
    - End-effector offset: (x, y, z, qx, qy, qz, qw)
    - Finger joint angle: thumb_fle, index_fle, mrl_fle

    :param grasp_point: Grasp point.
    :param object_name: Name of the object to be loaded: 'insole',
        'pillow_small', or 'box' (default: 'insole').
    :param horizon: Number of steps in simulation.
    :param train: Bool flag indicating if the starting position of the grasped
    :param compute_reward: Compute reward.
    :param object_scale: Scaling factor for object.
    :param task_space_limit: Limits for task space, shape: (2, 3).
    :param gui: Show PyBullet GUI (default: True).
    :param real_time: Run simulation in real time (default: False).
    :param initial_joint_positions: Initial positions of six finger joints
        (default: zeros).
    :param verbose: Print debug output (default: False).
    """
    def __init__(
            self,
            grasp_point: Union[npt.ArrayLike, None] = None,
            **kwargs):
        super(FloatingMiaGraspEnvWithMetrics, self).__init__(**kwargs)
        self.grasp_point = grasp_point

    def set_world_pose(self, world_pose):
        """Set pose of the hand.

        :param world_pose: world pose of hand given as position and
                           quaternion: (x, y, z, qw, qx, qy, qz)
        """
        self.hand_world_pose = np.hstack((
            world_pose[:3], pr.quaternion_xyzw_from_wxyz(world_pose[3:])))

    def get_world_pose(self):
        """Get pose of the hand.

        :return: hand pose in world coordinates given as position and
                 quaternion: (x, y, z, qw, qx, qy, qz)
        """
        return np.hstack((
            self.hand_world_pose[:3],
            pr.quaternion_wxyz_from_xyzw(self.hand_world_pose[3:])))

    def calculate_reward(self, state, action, next_state, done):
        """Calculates the reward given a (state,action,state) tuple.

        :param state: Current state of the Mia hand.
        :param action: Action performed by the Mia hand.
        :param next_state: Successor state of the Mia hand.
        :param done: Is the episode finished?
        """
        if done:
            return _final_reward_with_metrics(self, drop_test_time=1.0)
        else:
            return 0.0


def _final_reward_with_metrics(env, drop_test_time, verbose=1):
    if verbose:
        print("Episode done, calculating final reward with metrics...")

    Q_O, dist, dist_fingers = _approach_info(env)

    if hasattr(env, "invalid_initial_pose") and env.invalid_initial_pose:
        if verbose:
            print(f"Stage 1: {dist=:.3f}, {dist_fingers=:.3f}, {Q_O=:.2f}")
        return -15.0 - 5 * dist - 5 * dist_fingers + Q_O

    # before removing anchors
    contacts, _, _, _ = collect_contact_information(
        env.object_to_grasp.get_id(), env.robot.get_id(), 0,
        default_torque=1.0)
    n_contacts_before = len(contacts)
    initial_height = env.object_to_grasp.get_pose()[2]
    if n_contacts_before == 0:
        if verbose:
            print(f"Stage 2: {dist=:.3f}, {dist_fingers=:.3f}, {Q_O=:.2f}")
        return -10.0 - 5 * dist - 5 * dist_fingers + Q_O

    env.object_to_grasp.remove_anchors()
    env.simulation.simulate_time(drop_test_time)

    # after removing anchors
    contacts, vertices_in_object, _, object2world = \
        collect_contact_information(
            env.object_to_grasp.get_id(), env.robot.get_id(), 0,
            default_torque=1.0)
    vertices_in_world = pt.transform(
        object2world, pt.vectors_to_points(vertices_in_object))[:, :3]

    if len(contacts) == 0:
        if verbose:
            print(f"Stage 3: {dist=:.3f}, {dist_fingers=:.3f}, {Q_O=:.2f}, {n_contacts_before=}")
        return -5.0 - 5 * dist - 5 * dist_fingers + Q_O + 0.2 * n_contacts_before

    Q_FC, Q_SGP, Q_AGP, Q_DCC = _metrics(contacts, env, object2world, vertices_in_world)
    height = env.object_to_grasp.get_pose()[2]
    success = height > initial_height - 0.3
    success_reward = 5.0 if success else 0.0
    if verbose:
        print(f"Stage 4: {success=}, {height=:.2f}/{initial_height=:.2f}, "
              f"{dist=:.3f}, {Q_FC=:.5f}, {Q_DCC=:.3f}, {Q_AGP=:.3f}, "
              f"{Q_SGP=:.3f}, {Q_O=:.2f}, {dist_fingers=:.3f}")
    return success_reward - 5 * dist - 10 * dist_fingers + 200 * Q_FC + 0 * Q_DCC + 0 * Q_AGP + 0 * Q_SGP + 5 * Q_O


def _approach_info(env):
    hand = MiaHand()
    hand_pose = pt.transform_from_pq(env.get_world_pose())
    hand_pos, approach_direction = hand.perpendicular_to_palm_surface(hand_pose)
    I = compute_inertia(env.object_to_grasp.get_id())
    Q_O = orthogonality(I, approach_direction)
    approach_end = hand_pos + hand.get_maximum_finger_length() * approach_direction
    if env.verbose:
        pb.addUserDebugLine(hand_pos, approach_end, [1, 1, 0])
    if env.grasp_point is not None:
        dist = point_to_line_segment(
            env.grasp_point[:3], hand_pos, approach_end)[0]
        dist_fingers = distance_grasp_point_finger_tips(
            hand_pose, hand, env.grasp_point[:3])
    else:
        dist = 0.0
        dist_fingers = 0.0
    return Q_O, dist, dist_fingers


def _metrics(contacts, env, object2world, vertices_in_world):
    Q_FC = ferrari_canny(contacts)
    if env.grasp_point is None:
        Q_DCC = distance_com_centroid(
            contacts.contact_points_in_world(object2world),
            vertices_in_world)
    else:
        Q_DCC = distance_grasp_point_centroid(
            contacts.contact_points_in_world(object2world),
            vertices_in_world, env.grasp_point)
    if len(contacts) >= 3:
        normal, d, contact_points_in_plane = fit_plane_from_point_set(
            contacts.contact_points())
        Q_AGP = area_of_grasp_polygon(
            contact_points_in_plane, vertices_in_world, normal=normal)
        Q_SGP = grasp_polygon_shape(contact_points_in_plane, normal=normal)
    else:
        Q_AGP = 0.0
        Q_SGP = 0.0
    return Q_FC, Q_SGP, Q_AGP, Q_DCC
