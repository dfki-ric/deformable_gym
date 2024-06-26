from typing import Optional, Protocol, Union

import numpy as np
import numpy.typing as npt


class Sampler(Protocol):
    def sample_initial_pose(self) -> npt.NDArray: ...


class FixedSampler(Sampler):
    def __init__(self, initial_pose: npt.ArrayLike):
        self.initial_pose = np.array(initial_pose)

    def sample_initial_pose(self) -> npt.NDArray:
        return self.initial_pose


class GaussianSampler(Sampler):
    def __init__(
        self,
        mu: npt.ArrayLike,
        sigma: npt.ArrayLike,
        seed: Optional[int] = None,
    ):
        self.mu = np.array(mu)
        self.sigma = np.array(sigma)
        self.rng = np.random.default_rng(seed)

    def sample_initial_pose(self) -> npt.NDArray:
        return self.rng.normal(self.mu, self.sigma)


class UniformSampler(Sampler):
    def __init__(
        self,
        low: npt.ArrayLike,
        high: npt.ArrayLike,
        seed: Optional[int] = None,
    ):
        self.low = np.array(low)
        self.high = np.array(high)
        self.rng = np.random.default_rng(seed)

    def sample_initial_pose(self) -> npt.NDArray:
        return self.rng.uniform(self.low, self.high)


class GaussianCurriculumSampler(Sampler):
    def __init__(
        self,
        mu: npt.ArrayLike,
        sigma: npt.ArrayLike,
        step_size: Union[float, npt.ArrayLike] = 1e-3,
        seed: Optional[int] = None,
    ):
        self.mu = np.array(mu)
        self.sigma = np.array(sigma)
        self.rng = np.random.default_rng(seed)
        self.step_size = step_size

    def sample_initial_pose(self) -> npt.NDArray:
        self.sigma += self.step_size
        return self.rng.normal(self.mu, self.sigma)


class UniformCurriculumSampler(Sampler):
    def __init__(
        self,
        low: npt.ArrayLike,
        high: npt.ArrayLike,
        step_size: Union[float, npt.ArrayLike] = 1e-3,
        seed: Optional[int] = None,
    ):
        self.low = np.array(low)
        self.high = np.array(high)
        self.rng = np.random.default_rng(seed)
        self.step_size = step_size

    def sample_initial_pose(self) -> npt.NDArray:
        self.high += self.step_size
        self.low -= self.step_size
        return self.rng.uniform(self.low, self.high)


class GridSampler(Sampler):
    def __init__(
        self,
        low: npt.ArrayLike,
        high: npt.ArrayLike,
        n_points_per_axis: npt.ArrayLike,
    ):
        self.n_dims = len(low)
        points_per_axis = [
            np.linspace(low[i], high[i], n_points_per_axis[i])
            for i in range(self.n_dims)
        ]

        self.grid = np.array(np.meshgrid(*points_per_axis)).T.reshape(
            -1, self.n_dims
        )
        self.n_samples = len(self.grid)
        self.n_calls = 0

    def sample_initial_pose(self) -> npt.NDArray:
        sample = self.grid[self.n_calls % self.n_samples].copy()
        self.n_calls += 1
        return sample
