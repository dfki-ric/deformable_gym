from typing import List, Union


class PositionControlMixin:
    """Mixin for position-controlled robots."""

    velocity_commands = False

    def actuate_motors(self, keys: Union[list[str], None] = None):
        """
        Takes the provided action and sends the joint controls to the robot.

        :param keys: List of the motor keys to be actuated, if None all motor
        are actuated.
        """
        if keys is None:
            keys = self.motors.keys()
        for key in keys:
            self.motors[key].set_target_position(self.current_command[key])


class VelocityControlMixin:
    """Mixin for velocity-controlled robots."""

    velocity_commands = True

    def actuate_motors(self, keys: Union[list[str], None] = None):
        """
        Takes the provided action and sends the joint controls to the robot.

        :param keys: List of the motor keys to be actuated, if None all motor
        are actuated.
        """
        if keys is None:
            keys = self.motors.keys()
        for key in keys:
            self.motors[key].set_target_velocity(self.current_command[key])
