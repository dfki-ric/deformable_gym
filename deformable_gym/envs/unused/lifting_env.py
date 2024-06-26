import numpy as np
from gym import spaces

from deformable_gym.envs.base_env import BaseBulletEnv
from deformable_gym.helpers import pybullet_helper as pbh
from deformable_gym.objects.bullet_object import BoxObject
from deformable_gym.robots import ur5_mia


class LiftingEnv(BaseBulletEnv):
    """
    Basic grasping environment using the MIA hand. Goal is to grasp a basic
    rigid object (cube?) using joint velocity actions.
    """

    HORIZON = 10000

    def __init__(
        self,
        gui=True,
        real_time=False,
        initial_joint_positions=(
            0.0,
            -1.344,
            1.786,
            -2.646,
            -1.587,
            3.14,
            0,
            0,
            0,
            0,
            0,
            0,
        ),
        verbose=False,
    ):
        super().__init__(
            gui=gui,
            real_time=real_time,
            horizon=self.HORIZON,
            soft=True,
            load_plane=True,
            verbose=verbose,
        )

        self.robot = ur5_mia.UR5Mia(
            task_space_limit=(
                np.array([-0.8, -0.2, 1]),
                np.array([-0.3, 0.2, 1.4]),
            ),
            verbose=verbose,
        )

        self.robot.set_initial_joint_positions(
            np.array(initial_joint_positions)
        )

        self.step_counter = 0

        limits = pbh.get_limit_array(self.robot.motors)

        lower = np.concatenate(
            [
                np.array([-1.25, -0.25, 0.75]),
                np.zeros(4),
                limits[0][6:],
                np.array([-2, -2, -2]),
                np.array([2, 2, 2]),
            ],
            axis=0,
        )
        upper = np.concatenate(
            [
                np.array([-0.75, 0.25, 1.25]),
                np.ones(4),
                limits[1][6:],
                np.array([2, 2, 2]),
                np.array([2, 2, 2]),
            ],
            axis=0,
        )

        self.observation_space = spaces.Box(low=lower, high=upper)

        self.action_space = spaces.Box(
            low=-np.ones(11) * 0.05, high=np.ones(11) * 0.05
        )

        self.goal_space = spaces.Box(np.zeros(3), np.ones(3))

    def _load_objects(self):

        super()._load_objects()

        self.table = BoxObject(
            half_extents=np.array([0.5, 0.5, 0.5]),
            world_pos=[-0.5, 0.0, 0.5],
            world_orn=np.deg2rad([0, 0, 0]),
            mass=100,
            fixed=True,
        )

        self.cube = BoxObject(
            half_extents=np.array([0.05, 0.05, 0.05]),
            world_pos=[-0.5, 0.0, 1.05],
            world_orn=np.deg2rad([0, 0, 0]),
            mass=1,
            fixed=False,
        )

    def _hard_reset(self):
        super()._hard_reset()
        self.robot = ur5_mia.UR5Mia()

    def reset(self):
        self.cube.reset()
        return super().reset()

    def is_done(self, state, action, next_state):
        """
        Checks whether the current episode is over or not.
        """
        return (
            next_state[-1] < 0.85
            or next_state[-1] > 1.15
            or super().is_done(state, action, next_state)
        )

    def observe_state(self):
        joint_pos = self.robot.get_joint_positions()
        endeffector_pose = self.robot.get_ee_pose()
        sensor_readings = self.robot.get_sensor_readings()

        cube_pos = self.cube.get_pose()[:3]

        return np.concatenate(
            [endeffector_pose, joint_pos[6:], sensor_readings, cube_pos], axis=0
        ).astype("float64")

    def calculate_reward(self, state, action, next_state, done):
        """
        Calculates the reward by counting how many insole vertices are in the
        target position.
        """
        cube_pos = next_state[-3:]

        if cube_pos[2] < 0.85:
            return -100
        elif cube_pos[2] > 1.15:
            return 100
        else:
            return np.linalg.norm(next_state[2] - 1.05) * 0.01
