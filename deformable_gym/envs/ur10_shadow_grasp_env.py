import numpy as np
import pytransform3d.transformations as pt
from gymnasium import spaces

from deformable_gym.envs.base_env import BaseBulletEnv, GraspDeformableMixin
from deformable_gym.helpers import pybullet_helper as pbh
from deformable_gym.objects.bullet_object import ObjectFactory
from deformable_gym.robots import ur10_shadow


class UR10ShadowGraspEnv(GraspDeformableMixin, BaseBulletEnv):
    """Grasp an insole with UR10e + Shadow hand.

    **State space:**
    - End-effector pose: (x, y, z, qx, qy, qz, qw)
    - Finger joint angles: 6 values TODO order

    **Action space:**
    - End-effector pose offset: (x, y, z, qx, qy, qz, qw)
    - Finger joint angle offsets: 3 values

    Parameters
    ----------
    :param gui: Show PyBullet GUI.
    :param real_time: Simulate environment in real time.
    :param verbose: Verbose output.
    :param horizon: Number of steps in simulation.
    :param train: Bool flag indicating if the starting position of the grasped
    object should be sampled from the training or the test set.
    """

    object2world = pt.transform_from(R=np.eye(3), p=np.array([-0.7, 0.1, 1.8]))

    def __init__(
            self,
            gui=True,
            real_time=False,
            object_name="insole",
            verbose=False,
            horizon=100,
            object_scale=1.0,
            verbose_dt=10.0
    ):
        self.insole = None
        self.velocity_commands = False
        self.object_name = object_name
        self.object_scale = object_scale
        super().__init__(gui=gui, real_time=real_time, horizon=horizon,
                         soft=True, verbose=verbose,
                         verbose_dt=verbose_dt)

        self.robot = self._create_robot()

        limits = pbh.get_limit_array(self.robot.motors.values())

        lower_observations = np.concatenate([
            np.array([-2, -2, 0]), -np.ones(4), limits[0][6:],
            np.array([-5, -5, -5])], axis=0)

        upper_observations = np.concatenate([
            np.array([2, 2, 2]), np.ones(4), limits[1][6:],
            np.array([5, 5, 5])], axis=0)

        self.observation_space = spaces.Box(
            low=lower_observations, high=upper_observations)

        lower_actions = np.concatenate([
            np.array([-2, -2, 0]), -np.ones(4), limits[0][6:]], axis=0)

        upper_actions = np.concatenate([
            np.array([2, 2, 2]), np.ones(4), limits[1][6:]], axis=0)

        self.action_space = spaces.Box(low=lower_actions, high=upper_actions)

    def _create_robot(self):
        task_space_limit = None
        orn_limit = None
        # orn_limit = [[0, 0, 0], [0, 0, 0]]

        if self.velocity_commands:
            robot = ur10_shadow.UR10ShadowVelocity(
                task_space_limit=task_space_limit,
                end_effector_link="rh_forearm",
                verbose=self.verbose, orn_limit=orn_limit)
        else:
            robot = ur10_shadow.UR10ShadowPosition(
                task_space_limit=task_space_limit,
                end_effector_link="rh_forearm",
                verbose=self.verbose, orn_limit=orn_limit)

        robot.set_initial_joint_positions(dict(zip(robot.motors, robot.get_joint_positions())))

        self.simulation.add_robot(robot)

        return robot

    def _load_objects(self):
        super()._load_objects()
        self.object_to_grasp, self.object_position, self.object_orientation = \
            ObjectFactory().create(
                self.object_name,
                object2world=self.object2world,
                scale=self.object_scale)

    def reset(self, seed=None, options=None):

        self.object_to_grasp.reset()
        self.robot.activate_motors()

        return super().reset(seed, options)

    def _get_observation(self):
        finger_pos = self.robot.get_joint_positions()[6:]
        ee_pose = self.robot.get_ee_pose()
        sensor_readings = self.robot.get_sensor_readings()

        return np.concatenate([ee_pose, finger_pos, sensor_readings], axis=0)

    def calculate_reward(self, state, action, next_state, terminated):
        """
        Calculates the reward by counting how many insole vertices are in the
        target position.
        """
        if terminated:
            self.robot.deactivate_motors()
            # remove insole anchors and simulate steps
            self.object_to_grasp.remove_anchors()
            for _ in range(50):
                if self._deformable_is_exploded():
                    return -100
                self.simulation.step_to_trigger("time_step")
            height = self.object_to_grasp.get_pose()[2]
            if height > 0.9:
                return 1.0
            else:
                return -1.0
        else:
            return 0.0
