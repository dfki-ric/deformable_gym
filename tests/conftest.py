import pytest
import pybullet as pb
from deformable_gym.envs.bullet_simulation import BulletSimulation


@pytest.fixture
def simulation():
    sim = BulletSimulation(mode=pb.DIRECT, verbose_dt=10000, soft=True)
    sim.timing.add_subsystem("time_step", 100, None)
    return sim


@pytest.fixture
def mia_motors():
    return ["j_thumb_fle", "j_thumb_opp", "j_index_fle",  "j_mrl_fle", "j_ring_fle", "j_little_fle"]


@pytest.fixture
def mia_sensors():
    return ["j_index_sensor", "j_middle_sensor", "j_thumb_sensor"]


@pytest.fixture
def shadow_motors():
    return ["rh_WRJ2", "rh_WRJ1", "rh_THJ5", "rh_THJ4", "rh_THJ3", "rh_THJ2", "rh_THJ1", "rh_LFJ5", "rh_LFJ4",
            "rh_LFJ3", "rh_LFJ2", "rh_LFJ2", "rh_RFJ1", "rh_RFJ4", "rh_RFJ3", "rh_RFJ2", "rh_RFJ1", "rh_MFJ4",
            "rh_MFJ3", "rh_MFJ2", "rh_MFJ1", "rh_FFJ4", "rh_FFJ3", "rh_FFJ2", "rh_FFJ1"]


@pytest.fixture
def ur5_mia_motors():
    return ["ur5_shoulder_pan_joint", "ur5_shoulder_lift_joint", "ur5_elbow_joint", "ur5_wrist_1_joint",
            "ur5_wrist_2_joint", "ur5_wrist_3_joint", "j_thumb_fle", "j_thumb_opp", "j_index_fle",  "j_mrl_fle",
            "j_ring_fle", "j_little_fle"]


@pytest.fixture
def ur5_mia_sensors():
    return ["j_index_sensor", "j_middle_sensor", "j_thumb_sensor"]


@pytest.fixture
def ur10_shadow_motors():
    return ["ur10_shoulder_pan_joint", "ur10_shoulder_lift_joint", "ur10_elbow_joint", "ur10_wrist_1_joint",
            "ur10_wrist_2_joint", "ur10_wrist_3_joint", "rh_WRJ2", "rh_WRJ1", "rh_THJ5", "rh_THJ4", "rh_THJ3",
            "rh_THJ2", "rh_THJ1", "rh_LFJ5", "rh_LFJ4", "rh_LFJ3", "rh_LFJ2", "rh_LFJ1", "rh_RFJ1", "rh_RFJ4",
            "rh_RFJ3", "rh_RFJ2", "rh_RFJ1", "rh_MFJ4", "rh_MFJ3", "rh_MFJ2", "rh_MFJ1", "rh_FFJ4", "rh_FFJ3",
            "rh_FFJ2", "rh_FFJ1"]
