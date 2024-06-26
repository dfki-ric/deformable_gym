import numpy as np
import numpy.typing as npt
import pytest
from numpy.testing import assert_allclose

from deformable_gym.envs.sampler import (
    FixedSampler,
    GaussianSampler,
    GridSampler,
    UniformSampler,
)

SEED = 0


@pytest.fixture
def gaussian_target_pose() -> npt.NDArray:
    rng = np.random.default_rng(SEED)
    target = rng.normal(np.array([1, 2, 3]), np.array([2, 3, 4]))
    return target


@pytest.fixture
def fixed_target_pose() -> npt.NDArray:
    return np.array([1, 2, 3])


@pytest.fixture
def uniform_target_pose() -> npt.NDArray:
    rng = np.random.default_rng(SEED)
    target = rng.uniform(np.array([1, 2, 3]), np.array([2, 3, 4]))
    return target


@pytest.fixture
def grid_target_pose() -> npt.NDArray:
    target = np.array([1, 2, 3, 4])
    return target


@pytest.fixture
def fixed_sampler(fixed_target_pose: npt.NDArray) -> FixedSampler:
    return FixedSampler(fixed_target_pose)


@pytest.fixture
def gaussian_sampler() -> GaussianSampler:
    return GaussianSampler(
        mu=np.array([1, 2, 3]), sigma=np.array([2, 3, 4]), seed=SEED
    )


@pytest.fixture
def uniform_sampler() -> UniformSampler:
    return UniformSampler(
        low=np.array([1, 2, 3]), high=np.array([2, 3, 4]), seed=SEED
    )


@pytest.fixture
def grid_sampler() -> GridSampler:
    return GridSampler(
        low=np.array([1, 2, 3, 4]),
        high=np.array([2, 3, 4, 5]),
        n_points_per_axis=np.array([5, 3, 1, 1]),
    )


def test_fixed_sampler(
    fixed_sampler: FixedSampler, fixed_target_pose: npt.NDArray
):
    sampled_pose = fixed_sampler.sample_initial_pose()
    assert_allclose(sampled_pose, fixed_target_pose)


def test_gaussian_sampler(
    gaussian_sampler: GaussianSampler, gaussian_target_pose: npt.NDArray
):
    sampled_pose = gaussian_sampler.sample_initial_pose()
    assert_allclose(sampled_pose, gaussian_target_pose)


def test_uniform_sampler(
    uniform_sampler: UniformSampler, uniform_target_pose: npt.NDArray
):
    sampled_pose = uniform_sampler.sample_initial_pose()
    assert_allclose(sampled_pose, uniform_target_pose)


def test_grid_sampler(grid_sampler: GridSampler, grid_target_pose: npt.NDArray):
    sampled_pose = grid_sampler.sample_initial_pose()

    for i in range(20):
        print(grid_sampler.sample_initial_pose())

    assert len(grid_sampler.grid) == 15
    assert_allclose(sampled_pose, grid_target_pose)
