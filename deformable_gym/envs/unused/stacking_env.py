import numpy as np
from gym import spaces

from deformable_gym.envs.base_env import BaseBulletEnv
from deformable_gym.helpers import pybullet_helper as pbh
from deformable_gym.objects.bullet_object import BoxObject
from deformable_gym.robots import ur5_mia


class StackingEnv(BaseBulletEnv):
    """
    Basic grasping environment using the MIA hand. Goal is to grasp a basic rigid object (cube?) using joint velocity
    actions.
    """

    HORIZON = 2000

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
                np.array([-5, -5, -5]),
                np.array([5, 5, 5, 5, 5, 5]),
            ],
            axis=0,
        )
        upper = np.concatenate(
            [
                np.array([-0.75, 0.25, 1.25]),
                np.ones(4),
                limits[1][6:],
                np.array([5, 5, 5]),
                np.array([5, 5, 5, 5, 5, 5]),
            ],
            axis=0,
        )

        self.observation_space = spaces.Box(low=lower, high=upper)
        # initially: actions are joint targets
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

        self.cube1 = BoxObject(
            half_extents=np.array([0.05, 0.05, 0.05]),
            world_pos=[-0.5, -0.1, 1.05],
            world_orn=np.deg2rad([0, 0, 0]),
            mass=1,
            fixed=False,
        )

        self.cube2 = BoxObject(
            half_extents=np.array([0.05, 0.05, 0.05]),
            world_pos=[-0.5, 0.1, 1.05],
            world_orn=np.deg2rad([0, 0, 0]),
            mass=1,
            fixed=False,
        )

    def _hard_reset(self):
        super()._hard_reset()
        self.robot = ur5_mia.UR5Mia()

    def reset(self):
        self.cube1.reset()
        self.cube2.reset()
        return super().reset()

    def observe_state(self):
        joint_pos = self.robot.get_joint_positions()
        endeffector_pose = self.robot.get_ee_pose()
        sensor_readings = self.robot.get_sensor_readings()

        cube1_pos = self.cube1.get_pose()[:3]
        cube2_pos = self.cube2.get_pose()[:3]

        return np.concatenate(
            [
                endeffector_pose,
                joint_pos[6:],
                sensor_readings,
                cube1_pos,
                cube2_pos,
            ],
            axis=0,
        ).astype("float64")

    def calculate_reward(self, state, action, next_state, done):
        """
        Calculates the reward by counting how many insole vertices are in the target position.
        """
        cube2_pos = next_state[-3:]
        cube1_pos = next_state[-6:-3]

        if cube1_pos[2] < 0.5 or cube2_pos[2] < 0.5:
            return -100
        else:
            return (
                -np.linalg.norm(
                    self.cube1.get_pose()[:2] - self.cube2.get_pose()[:2]
                )
                * 0.1
            )
