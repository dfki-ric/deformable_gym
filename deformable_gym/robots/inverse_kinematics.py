import numpy as np
import pybullet as pb
from deformable_gym.robots.ur_kinematics import (
    analytical_ik, urdf_base2kin_base, ee_kin2ee_options, robot_params)


class PyBulletSolver:
    def __init__(
            self,
            robot,
            end_effector,
            n_iter: int = 200,
            threshold: float = 1e-6,
    ):
        self.robot = robot
        self.end_effector = end_effector
        self.n_iter = n_iter
        self.threshold = threshold

    def __call__(self, target_pos, target_orn, last_joint_angles):
        # target_orn = None

        if last_joint_angles is None:  # HACK
            last_joint_angles = np.zeros(6)

        out = np.array(pb.calculateInverseKinematics(
            self.robot,
            self.end_effector,
            targetPosition=target_pos,
            targetOrientation=target_orn,
            maxNumIterations=self.n_iter,
            residualThreshold=self.threshold,
            physicsClientId=self.robot.physics_client_id))
        return out[:6]


class UniversalRobotAnalyticalInverseKinematics:
    def __init__(
            self,
            robot_type="ur5",
            end_effector_name="ur5_tool0",
            fallback=None
    ):
        self.params = robot_params[robot_type]
        self.ee_kin2ee = ee_kin2ee_options[end_effector_name]
        self.fallback = fallback

    def __call__(self, target_pos, target_orn, last_joint_angles):
        if last_joint_angles is None:  # HACK
            last_joint_angles = np.zeros(6)

        joint_angles = analytical_ik(
            np.hstack((target_pos, target_orn)),
            last_joint_angles,
            params=self.params,
            urdf_base2kin_base=urdf_base2kin_base,
            ee_kin2ee=self.ee_kin2ee)

        if joint_angles is None:
            if self.fallback is None:
                return last_joint_angles
            else:
                return self.fallback(target_pos, target_orn, last_joint_angles)
        else:
            return joint_angles
