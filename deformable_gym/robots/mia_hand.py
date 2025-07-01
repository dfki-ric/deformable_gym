import abc
from typing import Dict, List, Tuple, Union

import numpy as np
import numpy.typing as npt
from pybullet_utils import bullet_client as bc

from ..helpers.pybullet_helper import Joint, load_urdf_from_resource
from ..robots.bullet_robot import BulletRobot, HandMixin, RobotCommandWrapper
from ..robots.control_mixins import PositionControlMixin, VelocityControlMixin
from ..robots.sensors import MiaHandForceSensors


class MiaHandMixin(HandMixin):
    # motor array, which includes all revolute joints, we omit j_thumb_opp,
    # j_thumb_opp cannot be controlled actively
    actuated_simulated_joints: list[str] = [
        "j_index_fle",
        "j_little_fle",
        "j_mrl_fle",
        "j_ring_fle",
        "j_thumb_fle",
    ]
    actuated_real_joints: list[str] = [
        "j_index_fle",
        "j_mrl_fle",
        "j_thumb_fle",
    ]

    mia_hand_force_sensors: MiaHandForceSensors

    motors: dict[str, Joint]

    def _correct_index_limit(self):
        """Correct the lower limit of index finger."""
        upper = self.motors["j_index_fle"].high
        self.motors["j_index_fle"].set_limits(0.0, upper)

    def set_thumb_opp(self, thumb_adducted: bool):
        """Set adduction / abduction of thumb.

        :param thumb_adducted: Is the thumb adducted (or abducted)?
        """
        if thumb_adducted:
            self.motors["j_thumb_opp"].set_position(0.0)
            self.motors["j_thumb_opp"].init_pos = 0.0
        else:
            self.motors["j_thumb_opp"].set_position(-0.628)
            self.motors["j_thumb_opp"].init_pos = -0.628

    def update_current_hand_command(
        self, motors: dict[str, Joint], command: npt.ArrayLike
    ):
        """Translates hand commands and updates current command.

        :param motors: Joints.
        :param command: 3D joint command array (fle_index, fle_mrl, fle_thumb)
        """
        assert (
            len(command) == 3
        ), f"expected command to have length 3, got {command=} instead"

        hand_joint_target = self.convert_action_to_pybullet(command)

        if hand_joint_target[1] > 0:
            hand_joint_target[1:4] = np.min(hand_joint_target[1:4])
        else:
            hand_joint_target[1:4] = np.max(hand_joint_target[1:4])

        hand_targets = {k: t for k, t in zip(motors, hand_joint_target)}

        self.update_current_command(hand_targets)

    def convert_action_to_pybullet(self, command):
        # set the same position for middle, ring, little finger
        mrl_command = command[1]
        hand_joint_target = np.insert(command, [1, 2], mrl_command)
        # fill j_thumb_opp with target value
        hand_joint_target = np.insert(
            hand_joint_target, 4, self.motors["j_thumb_opp"].init_pos
        )
        return hand_joint_target

    def get_joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        """Get joint limits.

        :return: Array of lower limits and array of upper limits.
        """
        return self._get_joint_limits(self.actuated_real_joints)

    def get_sensor_readings(self) -> np.ndarray:
        """Reads all sensors of the robot.

        :return: Order of measured forces:
        - (0) tangential force at middle finger,
        - (1) normal force at index finger,
        - (2) tangential force at index finger,
        - (3) tangential force at thumb,
        - (4) normal force at thumb,
        - (5) normal force at middle finger.
        """
        return self.mia_hand_force_sensors.measure().copy()

    def get_sensor_limits(self) -> tuple[np.ndarray, np.ndarray]:
        return self.mia_hand_force_sensors.get_limits()


class MiaHand(MiaHandMixin, BulletRobot, abc.ABC):
    """Controller for the robotic MIA hand.

    Allows easy to use actions (initially position control) to be sent to a
    PyBullet simulation. Simulation is build from a URDF model.

    The Mia hand is controlled with a frequency of 20Hz.

    :param verbose: Verbosity level.
    :param task_space_limit: Task space limits (position).
    :param orn_limit: Orientation limits.
    :param world_pos: Position of robot in world coordinates: (x, y, z).
    :param world_orn: Orientation of robot in world coordinates represented
    as Euler angles (roll, pitch, yaw; extrinsic concatenation).
    :param debug_visualization: Draw pose of robot and object and contact
    normals.
    :param base_commands: Control pose of the hand (add 7 components to action
    space).
    """

    def __init__(
        self,
        pb_client: bc.BulletClient,
        verbose: bool = False,
        task_space_limit: Union[npt.ArrayLike, None] = None,
        orn_limit: Union[npt.ArrayLike, None] = None,
        world_pos: npt.ArrayLike = (0, 0, 1),
        world_orn: npt.ArrayLike = (-np.pi / 8, np.pi, 0),
        debug_visualization: bool = True,
        **kwargs,
    ):
        urdf_path = load_urdf_from_resource(pb_client, "mia_hand.urdf")
        super().__init__(
            pb_client=pb_client,
            urdf_path=urdf_path,
            verbose=verbose,
            world_pos=world_pos,
            world_orn=world_orn,
            task_space_limit=task_space_limit,
            orn_limit=orn_limit,
            **kwargs,
        )
        self.debug_visualization = debug_visualization

        rcw = RobotCommandWrapper(self, self.actuated_simulated_joints)
        self.subsystems["hand"] = (20, rcw)

        self._correct_index_limit()

        self.mia_hand_force_sensors = MiaHandForceSensors(
            self._id, self._joint_name_to_joint_id, self.pb_client
        )

        self.thumb_adducted = True

        if self.debug_visualization:
            self._init_debug_visualizations()

    def perform_command(self, command: npt.ArrayLike):
        """Translates hand commands and updates current command.

        :param command: Joint command: (j_index_fle, j_mrl_fle, j_thumb_fle)
        """

        # check if the base of the robot is controlled
        if self.base_commands:
            hand_joint_target = command[7:]
            self.move_base(command[:7])
        else:
            hand_joint_target = command

        self.update_current_hand_command(self.motors, hand_joint_target)

    def reset(self, keys=None):
        """Resets all joints and sensors.

        :param keys: Names of joints to reset. Default is all.
        """
        super().reset(keys)
        self.mia_hand_force_sensors.reset()
        if self.debug_visualization:
            self.contact_normals = []

        self.set_thumb_opp(self.thumb_adducted)
        self.motors["j_thumb_opp"].max_force = 10000000


class MiaHandPosition(PositionControlMixin, MiaHand):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class MiaHandVelocity(VelocityControlMixin, MiaHand):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
