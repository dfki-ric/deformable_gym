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
    def __init__(
            self,
            mu: npt.ArrayLike,
            sigma: npt.ArrayLike,
            seed: int | None = None
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
            seed: int | None = None
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
            step_size: float | npt.ArrayLike = 1e-3,
            seed: int | None = None
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
            step_size: float | npt.ArrayLike = 1e-3,
            seed: int | None = None
    ):
        self.low = np.array(low)
        self.high = np.array(high)
        self.rng = np.random.default_rng(seed)
        self.step_size = step_size

    def sample_initial_pose(self) -> npt.NDArray:
        self.high += self.step_size
        self.low -= self.step_size
        return self.rng.uniform(self.low, self.high)
