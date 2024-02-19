from typing import Protocol
import numpy.typing as npt
import numpy as np


class Sampler(Protocol):
    def sample_initial_pose(self) -> npt.NDArray:
        ...


class FixedSampler(Sampler):
    def __init__(self, initial_pose: npt.ArrayLike):
        self.initial_pose = np.array(initial_pose)

    def sample_initial_pose(self) -> npt.NDArray:
        return self.initial_pose


class GaussianSampler(Sampler):
    def __init__(self, mu: npt.ArrayLike, sigma: npt.ArrayLike):
        self.mu = np.array(mu)
        self.sigma = np.array(sigma)

    def sample_initial_pose(self) -> npt.NDArray:
        return np.random.normal(self.mu, self.sigma)


class UniformSampler(Sampler):
    def __init__(self, low: npt.ArrayLike, high: npt.ArrayLike):
        self.low = np.array(low)
        self.high = np.array(high)

    def sample_initial_pose(self) -> npt.NDArray:
        return np.random.uniform(self.low, self.high)


class GaussianCurriculumSampler(Sampler):
    def __init__(
            self,
            mu: npt.ArrayLike,
            sigma: npt.ArrayLike,
            step_size: float = 1e-3):
        self.mu = np.array(mu)
        self.sigma = np.array(sigma)
        self.step_size = step_size

    def sample_initial_pose(self) -> npt.NDArray:
        self.sigma += self.step_size
        return np.random.normal(self.mu, self.sigma)
