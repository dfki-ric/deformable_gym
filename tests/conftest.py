import pytest
import pybullet as pb
from deformable_gym.envs.pybullet_tools import BulletSimulation


@pytest.fixture
def simulation():
    sim = BulletSimulation(mode=pb.DIRECT, verbose_dt=10000, soft=True)
    sim.timing.add_subsystem("time_step", 100, None)
    return sim

