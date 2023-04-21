import numpy as np

from deformable_gym.envs.ur5_mia_grasp_env import UR5MiaGraspEnv, INITIAL_JOINT_ANGLES
from deformable_gym.robots.mia_hand import MiaHandMixin
from deformable_gym.helpers.state_converter import VectorConverter, TransformedDMP, ActionCoupling


class AprilInsoleDmpEnv(UR5MiaGraspEnv):
    """Control UR5 and Mia hand with a DMP.

    **State space:**
    The state space has 16 dimensions.
    The pose of the end-effector (7 values), finger joint angles (3 values),
    and sensor readings (6 values).
    End-effector poses are defined by position (x, y, z) and scalar-last
    quaternion (qx, qy, qz, qw).
    Joint positions are ordered as follows:
    - (0) j_index_fle
    - (2) j_mrl_fle
    - (5) j_thumb_fle
    Force measurements are ordered as follows:
    - (0) tangential force at middle finger,
    - (1) normal force at index finger,
    - (2) tangential force at index finger,
    - (3) tangential force at thumb,
    - (4) normal force at thumb,
    - (5) normal force at middle finger.

    **Action space:**
    The action space has 11 dimensions. These are coupling term values for
    the DMP: 7 values for the end-effector pose (7 values) and 3 values for
    the joint angles of the hand.
    End-effector commands are defined by position (x, y, z) and scalar-last
    quaternion (qx, qy, qz, qw).
    Joint commands are ordered as follows:
    - (0) j_index_fle
    - (1) j_mrl_fle
    - (3) j_thumb_fle
    """
    def __init__(
            self, dmp, gui=True, real_time=False,
            verbose=False, object2world=None, horizon=200):
        self.object2world = object2world
        super(AprilInsoleDmpEnv, self).__init__(
            gui=gui, real_time=real_time, verbose=verbose, horizon=horizon)
        self.dmp = dmp
        self.transformed_dmp = TransformedDMP(dmp, object2world)
        self.dmp_state2env_action = VectorConverter(
            ["x", "y", "z", "qw", "qx", "qy", "qz", "j_index_fle", "j_mrl_fle",
             "j_thumb_fle"],
            ["x", "y", "z", "qx", "qy", "qz", "qw", "j_index_fle", "j_mrl_fle",
             "j_thumb_fle"])
        self.env_state2dmp_state = VectorConverter(
            ["x", "y", "z", "qx", "qy", "qz", "qw", "j_index_fle", "j_mrl_fle",
             "j_middle_fle", "j_ring_fle", "j_thumb_opp", "j_thumb_fle",
             "sensor1", "sensor2", "sensor3"],
            ["x", "y", "z", "qw", "qx", "qy", "qz", "j_index_fle", "j_mrl_fle",
             "j_thumb_fle"])

    def reset(self, hard_reset=False):
        start_y = self.dmp_state2env_action(self.transformed_dmp.start_y)
        assert len(start_y) == 10
        arm_joint_target = self.robot.compute_ik(
            np.array(INITIAL_JOINT_ANGLES), start_y[:3], start_y[3:7], False)
        hand_joint_target = MiaHandMixin.convert_action_to_pybullet(start_y[7:])
        joint_angles = np.hstack((arm_joint_target[:6], hand_joint_target))
        self.robot.set_initial_joint_positions(joint_angles)
        return super().reset(hard_reset)

    def step(self, action):
        # observe current state
        state = self.observe_state()

        # translate action
        y = self.env_state2dmp_state(state)
        next_y = self.transformed_dmp.step(
            y, coupling_term=ActionCoupling(action))
        action = self.dmp_state2env_action(next_y)

        # execute action
        self.robot.perform_command(action)

        # simulate until next time step
        self.simulation.step_to_trigger("time_step")

        # observe new state
        next_state = self.observe_state()

        # update time step
        self.step_counter += 1

        # check if episode is over
        done = self.is_done(state, action, next_state)

        # calculate the reward
        reward = self.calculate_reward(state, action, next_state, done)

        return next_state, reward, done, {}
