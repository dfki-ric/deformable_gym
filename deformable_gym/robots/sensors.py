import matplotlib.pyplot as plt
import numpy as np
from pybullet_utils import bullet_client as bc

INDEX_SENSOR_NAME = "j_index_sensor"
MIDDLE_SENSOR_NAME = "j_middle_sensor"
THUMB_SENSOR_NAME = "j_thumb_sensor"
MM_PER_M = 1000.0


class MiaHandForceSensors:
    """PyBullet simulation of the Mia hand's force sensors.

    There are two force sensors attached to each the index finger, middle
    finger, and thumb, resulting in a total number of 6 sensors. The tangential
    and normal forces acting on each of the three fingers will be measured.

    Each sensor output is read through a proper electronic circuitry, then it
    is converted to digital domain by a 10-bit ADC. So the values read from
    the sensors before postprocessing and conversion to Newton are mapped to
    the [0, 1023] range. For tangential forces, you should expect a maximum
    output of approximately 570 for the thumb sensor, and approximately 408 for
    index and middle sensors. However, each Mia hand force sensor output is
    subject to an offset. When no forces are applied to the finger, the raw
    output read directly from the sensor is not 0, but a value between 0 and
    1023 (e.g., 700). This offset is different for each sensor. When a force is
    applied the raw output increases or decreases from this offset, according
    to the direction of the applied force. For the thumb sensor the positive
    direction of force is the one pointing at the finger front. This means that
    by pushing against finger front (e.g., during grasp), the raw sensor output
    will increase from the initial offset. On the contrary, by pushing against
    the back of the thumb, raw output will decrease. For what concerns index
    and middle sensors, the positive force direction is opposite with respect
    to thumb. This means that, by pushing against the finger front (e.g.,
    during grasp), the raw sensor output will decrease from the initial offset,
    while, by pushing against the back of the finger, it will increase.

    Prensilia added the option to read adjusted sensor outputs from the Mia
    Hand. With this option, sensor outputs are already streamed without offset,
    i.e., they are 0 when no forces are applied. We try to reproduce this
    behaviour in simulation. Hence, the values that this simulated force sensor
    measures are deviations from these initial offsets and they can be negative
    and positive. Moreover, by using this option, force directions are adjusted
    such that for all the fingers positive outputs are obtained by pushing
    against the front sides of the fingers. Index and middle force directions
    are reversed. This means that during grasps of objects, outputs read from
    force sensors should always be positive. Negative outputs will be obtained
    instead by pushing against the finger's back. Processing of Mia Hand force
    sensor outputs is simpler with these post-processing steps.

    Order of force measurements:
    - (0) tangential force at middle finger,
    - (1) normal force at index finger,
    - (2) tangential force at index finger,
    - (3) tangential force at thumb,
    - (4) normal force at thumb,
    - (5) normal force at middle finger.

    Parameters
    ----------
    robot_id : int
        Multibody unique ID identifying the robot.

    joint_name_to_joint_id : dict
        Mapping from joint names to joint IDs.

    debug : bool, optional (default: False)
        Sensor values of an episode will be stored in debug mode.
    """

    SENSOR_NAMES = [
        "middle tangential",
        "index normal",
        "index tangential",
        "thumb tangential",
        "thumb normal",
        "middle normal",
    ]

    def __init__(
        self,
        robot_id: int,
        joint_name_to_joint_id: dict,
        pb_client: bc.BulletClient,
        debug: bool = False,
    ):
        self.robot_id = robot_id
        self.index_sensor_id = joint_name_to_joint_id[INDEX_SENSOR_NAME]
        self.middle_sensor_id = joint_name_to_joint_id[MIDDLE_SENSOR_NAME]
        self.thumb_sensor_id = joint_name_to_joint_id[THUMB_SENSOR_NAME]
        self.debug = debug
        self.pb_client = pb_client

        for sensor_id in [
            self.index_sensor_id,
            self.middle_sensor_id,
            self.thumb_sensor_id,
        ]:
            self.pb_client.enableJointForceTorqueSensor(
                self.robot_id, sensor_id
            )

        if self.debug:
            self.history = []

    def reset(self):
        """Reset force sensors before new episode."""
        if self.debug and self.history:
            history = np.array(self.history)
            plt.figure()
            for i in range(history.shape[1]):
                plt.plot(history[:, i], label=self.SENSOR_NAMES[i])
            plt.legend()
            plt.show()
            self.history = []

    def get_limits(self):
        """Get lower and upper limits of sensor readings.

        Returns
        -------
        lower : array, shape (6,)
            Lower limits of sensor values.

        upper : array, shape (6,)
            Upper limits of sensor values.
        """
        return (
            np.array([-204.0, -512.0, -204.0, -285.0, -512.0, -512.0]),
            np.array([204.0, 512.0, 204.0, 285.0, 512.0, 512.0]),
        )

    def measure(self, out=None):
        """Take a force measurement.

        Parameters
        ----------
        out : array, shape (6,), optional (default: None)
            Output array to which the measurement will be written. A new one
            will be created if none is given.

        Returns
        -------
        out : array, shape (6,)
            Force measurements.
        """
        if out is None:
            out = np.empty(6)

        _, _, _, T_ind_x, _, T_ind_z = -np.array(
            self.pb_client.getJointState(
                self.robot_id,
                self.index_sensor_id,
            )[2]
        )
        out[1] = T_ind_z * MM_PER_M / 2.006
        out[2] = T_ind_x * MM_PER_M / 2.854

        _, _, _, T_mrl_x, _, T_mrl_z = -np.array(
            self.pb_client.getJointState(
                self.robot_id,
                self.middle_sensor_id,
            )[2]
        )
        out[5] = T_mrl_z * MM_PER_M / 2.006
        out[0] = T_mrl_x * MM_PER_M / 2.854

        _, F_th_y, _, _, T_th_y, _ = -np.array(
            self.pb_client.getJointState(
                self.robot_id,
                self.thumb_sensor_id,
            )[2]
        )
        out[3] = T_th_y * MM_PER_M / 3.673
        out[4] = F_th_y / 0.056

        # post-processing
        # make normal forces that occur during grasping positive
        out[1] *= -1.0  # index normal
        out[5] *= -1.0  # mrl normal
        # apply limits
        lower, upper = self.get_limits()
        np.clip(out, lower, upper, out=out)

        if self.debug:
            self.history.append(np.copy(out))

        return out
