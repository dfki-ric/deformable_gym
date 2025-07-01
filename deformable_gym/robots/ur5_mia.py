import abc

import numpy.typing as npt
from pybullet_utils import bullet_client as bc

from ..helpers.pybullet_helper import load_urdf_from_resource
from ..robots.bullet_robot import BulletRobot, RobotCommandWrapper
from ..robots.control_mixins import PositionControlMixin, VelocityControlMixin
from ..robots.inverse_kinematics import (
    UniversalRobotAnalyticalInverseKinematics,
)
from ..robots.sensors import MiaHandForceSensors
from .mia_hand import MiaHandMixin

# Mia freq = 20 Hz
# UR5 freq = 125 Hz


class UR5Mia(MiaHandMixin, BulletRobot, abc.ABC):
    """Controller for the robotic MIA hand.

    Allows easy to use actions (initially position control) to be sent to a
    PyBullet simulation. Simulation is build from a URDF model.

    TODO Can (and should) be extended eventually to allow other control
    methods, e.g. velocity or torque control.
    """

    def __init__(
        self,
        pb_client: bc.BulletClient,
        verbose: bool = False,
        task_space_limit=None,
        end_effector_link="ur5_tool0",
        orn_limit=None,
        debug_visualization=True,
    ):
        urdf_path = load_urdf_from_resource(pb_client, "mia_hand_on_ur5.urdf")
        super().__init__(
            urdf_path=urdf_path,
            pb_client=pb_client,
            verbose=verbose,
            task_space_limit=task_space_limit,
            orn_limit=orn_limit,
        )

        self.debug_visualization = debug_visualization

        self._arm_motors = {k: v for k, v in self.motors.items() if "ur5_" in k}
        self._hand_motors = {k: v for k, v in self.motors.items() if "j_" in k}

        hand_command_wrapper = RobotCommandWrapper(
            self, self.actuated_simulated_joints
        )
        self.subsystems["hand"] = (20, hand_command_wrapper)
        arm_command_wrapper = RobotCommandWrapper(self, self._arm_motors)
        self.subsystems["arm"] = (125, arm_command_wrapper)

        self.mia_hand_force_sensors = MiaHandForceSensors(
            self._id, self._joint_name_to_joint_id, self.pb_client
        )

        if self.debug_visualization:
            self._init_debug_visualizations()

        self.set_endeffector(self._link_name_to_link_id[end_effector_link])

        self.set_inverse_kinematics_solver(
            UniversalRobotAnalyticalInverseKinematics(
                end_effector_name=end_effector_link,
                fallback=self.inverse_kinematics_solver,
            )
        )

        self._correct_index_limit()

        self.motors["j_thumb_opp"].deactivate()

    def perform_command(self, command):
        """Performs the provided arm and hand commands.

        Assumes task-space arm command and joint-space hand command.

        :param command: End-effector pose and hand joint angles:
        (x, y, z, qx, qy, qz, qw, j_index_fle, j_mrl_fle, j_thumb_fle)
        """
        self.command_to_joint_targets(command)

    def command_to_joint_targets(self, command: npt.ArrayLike):
        assert (
            len(command) == 10
        ), f"expected command to have shape 10, got {len(command)} instead"
        # get arm joint targets
        self.arm_command_to_joint_targets(command[:7])
        # get hand joint actions
        self.hand_command_to_joint_targets(command[7:])

    def arm_command_to_joint_targets(self, command: npt.ArrayLike):
        """Converts task-space command for UR5 arm to joint targets.

        :param command: End-effector pose (x, y, z, qx, qy, qz, qw)
        target positions or velocities
        """
        assert (
            len(command) == 7
        ), f"expected command to have shape 7, got {len(command)} instead"

        # convert arm command to joint motor control
        arm_joint_angles = self.get_joint_positions(self._arm_motors)

        arm_joint_target = self.compute_ik(
            arm_joint_angles, command[:3], command[3:], False
        )

        targets = {k: t for k, t in zip(self._arm_motors, arm_joint_target)}

        self.update_current_command(targets)

    def hand_command_to_joint_targets(self, command: npt.ArrayLike):
        """Translates hand commands and updates current command.

        :param command: Joint angles (j_index_fle, j_mrl_fle, j_thumb_fle)
        """
        self.update_current_hand_command(self._hand_motors, command)

    def reset(self, keys=None):
        """Resets all joints and sensors.

        :param keys: Names of joints to reset. Default is all.
        """
        super().reset(keys)
        self.mia_hand_force_sensors.reset()
        if self.debug_visualization:
            self.contact_normals = []


class UR5MiaPosition(PositionControlMixin, UR5Mia):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)


class UR5MiaVelocity(VelocityControlMixin, UR5Mia):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
