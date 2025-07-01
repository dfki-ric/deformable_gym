import abc
from typing import Any

import numpy as np
from pybullet_utils import bullet_client as bc

from ..helpers.pybullet_helper import load_urdf_from_resource
from ..robots.bullet_robot import BulletRobot, HandMixin
from ..robots.control_mixins import PositionControlMixin, VelocityControlMixin
from ..robots.inverse_kinematics import (
    UniversalRobotAnalyticalInverseKinematics,
)

# Shadow freq = 500 Hz
# UR5 freq = 125 Hz


class UR10Shadow(HandMixin, BulletRobot, abc.ABC):
    """Controller for the robotic Shadow dexterous hand on UR10 robot arm.

    Allows easy to use actions (initially position control) to be sent to a
    PyBullet simulation. Simulation is build from a URDF model.

    TODO Can (and should) be extended eventually to allow other control
    methods, e.g. velocity or torque control.
    """

    def __init__(
        self,
        pb_client: bc.BulletClient,
        verbose: bool = False,
        task_space_limit: Any = None,
        end_effector_link: str = "rh_forearm",
        orn_limit=None,
        debug_visualization: bool = True,
    ):
        urdf_path = load_urdf_from_resource(
            pb_client, "shadow_hand_on_ur10.urdf"
        )
        super().__init__(
            urdf_path=urdf_path,
            pb_client=pb_client,
            verbose=verbose,
            task_space_limit=task_space_limit,
            orn_limit=orn_limit,
        )

        self.command_counter = 0
        self.debug_visualization = debug_visualization

        self._arm_motors = {
            k: v for k, v in self.motors.items() if "ur10_" in k
        }
        self._hand_motors = {k: v for k, v in self.motors.items() if "rh_" in k}

        def hand_command_wrapper():
            self.send_current_command(self._hand_motors)

        def arm_command_wrapper():
            self.send_current_command(self._arm_motors)

        self.subsystems["hand"] = (500, hand_command_wrapper)
        self.subsystems["arm"] = (125, arm_command_wrapper)

        if self.debug_visualization:
            self._init_debug_visualizations()

        self.set_endeffector(self._link_name_to_link_id[end_effector_link])

        self.set_inverse_kinematics_solver(
            UniversalRobotAnalyticalInverseKinematics(
                robot_type="ur10",
                end_effector_name=end_effector_link,
                fallback=self.inverse_kinematics_solver,
            )
        )

    def perform_command(self, command):
        """
        Performs the provided arm and hand commands. Assumes task-space arm
        command and joint-space hand command.
        """

        assert (
            command.shape[0] == 31
        ), f"""expected command to have shape 31, got {
            command.shape[0]
        } instead"""

        # get arm joint targets
        arm_target = self.arm_command_to_joint_targets(command[:7], False)
        super().update_current_command(arm_target)

        # get hand joint actions
        hand_target = self.hand_command_to_joint_targets(command[7:], False)
        super().update_current_command(hand_target)

    def arm_command_to_joint_targets(self, command, velocity_commands=False):
        """Converts task-space command for UR5 arm to joint targets.

        :param command: 7D end-effector pose (x, y, z, qx, qy, qz, qw)
        :param velocity_commands: boolean indicating if the command represents
        target positions or velocities
        :return: 6D array of UR10 target joint positions
        """
        assert (
            command.shape[0] == 7
        ), f"expected command to have shape 7, got {command.shape[0]} instead"

        # convert arm command to joint motor control
        arm_joint_angles = self.get_joint_positions(self._arm_motors)

        arm_joint_target = self.compute_ik(
            arm_joint_angles, command[:3], command[3:], velocity_commands
        )

        target_dict = {k: t for k, t in zip(self._arm_motors, arm_joint_target)}

        return target_dict

    def hand_command_to_joint_targets(self, command, velocity_commands=False):
        """Converts command for Shadow hand to correct joint targets.

        :param command: 24D joint command array
        :param velocity_commands: boolean indicating if the command represents
        target positions or velocities
        :return: 24D array of Mia hand target joint positions
        """
        assert (
            command.shape[0] == 24
        ), f"""expected command to have shape 24, got {
            command.shape[0]
        } instead"""

        if velocity_commands:
            current_joint_angles = self.get_joint_positions(self._hand_motors)
        else:
            current_joint_angles = np.zeros(24)

        # add hand joint actions
        hand_joint_target = current_joint_angles + command

        target_dict = {
            k: t for k, t in zip(self._hand_motors, hand_joint_target)
        }

        return target_dict


class UR10ShadowPosition(PositionControlMixin, UR10Shadow):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class UR10ShadowVelocity(VelocityControlMixin, UR10Shadow):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
