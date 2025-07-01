import abc
from typing import Union

import numpy as np
import numpy.typing as npt
from pybullet_utils import bullet_client as bc

from ..helpers.pybullet_helper import load_urdf_from_resource
from ..robots.bullet_robot import BulletRobot, HandMixin, RobotCommandWrapper
from ..robots.control_mixins import PositionControlMixin, VelocityControlMixin


class ShadowHand(HandMixin, BulletRobot, abc.ABC):
    """Controller for the robotic Shadow dexterous hand.

    Allows easy to use actions to be sent to a PyBullet simulation. Simulation
    is build from a URDF model.

    The Shadow hand is controlled with a frequency of 500 Hz.

    Note: Can (and should) be extended eventually to allow other control
    methods, e.g. velocity or torque control.

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
        urdf_path = load_urdf_from_resource(pb_client, "shadow_hand.urdf")
        super().__init__(
            urdf_path=urdf_path,
            pb_client=pb_client,
            verbose=verbose,
            world_pos=world_pos,
            world_orn=world_orn,
            task_space_limit=task_space_limit,
            orn_limit=orn_limit,
            **kwargs,
        )
        self.debug_visualization = debug_visualization

        hand_command_wrapper = RobotCommandWrapper(self, self.motors)
        self.subsystems["hand"] = (500, hand_command_wrapper)

        if self.debug_visualization:
            self._init_debug_visualizations()

    def perform_command(self, command: npt.ArrayLike):
        """Translates hand commands and updates current command.

        :param command: Joint angles
        """
        # check if the base of the robot is controlled
        if self.base_commands:
            hand_joint_target = command[7:]
            self.move_base(command[:7])
        else:
            hand_joint_target = command

        # update current robot command
        hand_targets = {k: t for k, t in zip(self.motors, hand_joint_target)}
        self.update_current_command(hand_targets)


class ShadowHandPosition(PositionControlMixin, ShadowHand):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class ShadowHandVelocity(VelocityControlMixin, ShadowHand):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
