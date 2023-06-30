import pybullet as pb
from enum import Enum


class JointType(Enum):
    revolute = pb.JOINT_REVOLUTE
    prismatic = pb.JOINT_PRISMATIC
    fixed = pb.JOINT_FIXED
    planar = pb.JOINT_PLANAR
    spherical = pb.JOINT_SPHERICAL


class Joint:

    def __init__(self, name, joint_idx, joint_type, pos_idx, vel_idx, low, high,
                 max_force, max_vel, body_id):
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

    def __repr__(self):
        return f"Joint({', '.join([f'{k}={v}' for k,v in vars(self).items()])})"

    def reset(self):
        """Resets the joint to its initial position."""
        self.set_position(self.init_pos)

    def set_position(self, position):
        """Sets the joint to target position."""
        pb.resetJointState(self.body_id, self.joint_idx,
                           targetValue=position, targetVelocity=0.0)

    def set_limits(self, low, high):
        """Sets the joint limits."""
        pb.changeDynamics(self.body_id, self.joint_idx,
                          jointLowerLimit=low, jointUpperLimit=high)

    def get_position(self):
        """Gets the current position of the joint."""
        return pb.getJointState(self.body_id, self.joint_idx)[0]

    def get_velocity(self):
        """Gets the current velocity of the joint."""
        return pb.getJointState(self.body_id, self.joint_idx)[1]

    def set_target_position(self, position: float):
        """Sets the target position of the joint."""

        if self.max_vel + 0.1 < self.get_velocity():
            if self.verbose:
                print(f"Joint {self.name} has max velocity of {self.max_vel}"
                      f" but is moving with velocity {self.get_velocity()}")

        if self.activated:
            pb.setJointMotorControl2(self.body_id, self.joint_idx,
                                     controlMode=pb.POSITION_CONTROL,
                                     targetPosition=position,
                                     targetVelocity=0.0,
                                     maxVelocity=self.max_vel,
                                     force=self.max_force)

        else:
            if self.verbose:
                print(f"Warning: Trying to control deactivated motor {self.name}.")

    def set_target_velocity(self, velocity: float):
        """Sets the target position of the joint."""
        if self.activated:
            pb.setJointMotorControl2(self.body_id, self.joint_idx,
                                     controlMode=pb.VELOCITY_CONTROL,
                                     targetVelocity=velocity,
                                     maxVelocity=self.max_vel,
                                     force=self.max_force)
        else:
            if self.verbose:
                print(f"Warning: Trying to control deactivated motor {self.name}.")

    def deactivate(self):
        """Deactivates the motor."""
        self.set_target_velocity(0.0)
        self.activated = False

    def activate(self):
        """Activates the motor."""
        self.activated = True
