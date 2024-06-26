import numpy as np
from gym import spaces

from deformable_gym.envs.base_env import BaseBulletEnv
from deformable_gym.helpers import pybullet_helper as pbh
from deformable_gym.objects.bullet_object import UrdfObject
from deformable_gym.robots import ur5_mia

VERBOSITY = 0


class UR5MiaReachEnv(BaseBulletEnv):
    """
    Basic grasping environment using the MIA hand. Goal is to grasp a basic
    rigid object (cube?) using joint velocity
    actions.
    """

    HORIZON = 200
    velocity_penalty = 0.0

    def __init__(self, gui=True, real_time=False):
        super().__init__(gui=gui, real_time=real_time, horizon=self.HORIZON)

        self.robot = ur5_mia.UR5MiaPosition()
        self.step_counter = 0

        limits = pbh.get_limit_array(self.robot.motors.values())

        lower = np.concatenate(
            [
                np.array([-1.25, -0.25, 0.75]),
                np.zeros(4),
                np.array([-1.5, -0.5, 0.5]),
                limits[0][6:],
                np.array([-5, -5, -5]),
            ],
            axis=0,
        )
        upper = np.concatenate(
            [
                np.array([-0.75, 0.25, 1.25]),
                np.ones(4),
                np.array([-0.5, 0.5, 1.5]),
                limits[1][6:],
                np.array([5, 5, 5]),
            ],
            axis=0,
        )

        self.observation_space = spaces.Box(low=lower, high=upper)
        # initially: actions are joint targets
        self.action_space = spaces.Box(low=-np.ones(11), high=np.ones(11))

        # define goal

        self.table_cube = UrdfObject(
            "cube_small.urdf", world_pos=[-0.75, 0.0, 0.25], fixed=True
        )
        self.goal_cube = UrdfObject(
            "cube_small.urdf", world_pos=[-0.75, 0.0, 0.751], fixed=False
        )

    def reset(self, reset_robot=True):
        self.goal_cube.reset()
        return super().reset()

    def observe_state(self):
        joint_pos = self.robot.get_joint_positions()
        endeffector_pose = self.robot.get_ee_pose()
        sensor_readings = self.robot.get_sensor_readings()
        cube_pos = self.goal_cube.get_pose()[:3]

        # print(sensor_readings)

        return np.concatenate(
            [endeffector_pose, cube_pos, joint_pos[6:], sensor_readings], axis=0
        )

    def calculate_reward(self, state, action, next_state, done):
        if next_state[9] < 0.5:
            return -10
        elif next_state[9] >= 1.0:
            return 10
        else:
            return -0.1 * np.linalg.norm(
                next_state[7:9] - np.array([-0.5, 0.5])
            )
