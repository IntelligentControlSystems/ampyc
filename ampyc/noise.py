'''
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2025, Intelligent Control Systems Group, ETH Zurich
%
% This code is made available under an MIT License (see LICENSE file).
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
'''

from abc import ABC, abstractmethod
import numpy as np

from ampyc.utils import Polytope, qhull

class NoiseBase(ABC):
    """Base class for random noise/disturbance generators"""

    state_dependent: bool = False
    rng: np.random.Generator | None = None

    def generate(self, N: int | None = None) -> np.ndarray:
        return self._generate(N)

    @classmethod
    @abstractmethod
    def _generate(self, N: int | None = None) -> np.ndarray:
        """Noise generation method to be implemented by the inherited class"""
        raise NotImplementedError

    def reset(self, *args, **kwargs):
        '''Calls the constructor again'''
        self.__init__(*args, **kwargs)
    
    def seed(self, seed: int | None = None):
        '''Resets the random number generator with a new seed'''
        assert seed is None or seed >= 0
        self.rng = np.random.default_rng(seed)


class ZeroNoise(NoiseBase):
    """Outputs zero noise, i.e., no disturbance"""

    def __init__(self, dim: int) -> None:
        self.dim = dim

    def _generate(self, N: int | None = None) -> np.ndarray:
        sample = np.zeros((self.dim, 1)) if N is None else np.zeros((self.dim, N))
        return sample


class GaussianNoise(NoiseBase):
    """Computes Gaussian disturbance based on noise mean and covariance"""

    def __init__(self, mean: np.ndarray, covariance: np.ndarray, seed: int | None = None) -> None:
        assert len(covariance.shape) == 2 and covariance.shape[0] == covariance.shape[1]
        assert len(mean) == covariance.shape[0]
        assert seed is None or seed >= 0
        self.mean = mean.reshape(-1)
        self.cov = covariance
        self.rng = np.random.default_rng(seed)

    def _generate(self, N: int | None = None) -> np.ndarray:
        if N is None:
            sample = self.rng.multivariate_normal(
                self.mean, self.cov, check_valid="raise"
            ).reshape(-1,1)
        else:
            sample = self.rng.multivariate_normal(
                self.mean, self.cov, size=(N,), check_valid="raise"
            ).T
        return sample


class TruncGaussianNoise(GaussianNoise):
    """Computes Gaussian disturbance based on noise mean and covariance in the set A_w * w <= b_w"""

    def __init__(self, mean: np.ndarray, covariance: np.ndarray, W: Polytope, max_iters: int = 1e4, seed: int | None = None) -> None:
        assert seed is None or seed >= 0
        super().__init__(mean, covariance, seed=seed)
        self.trunc_bounds = W
        self.max_iters = max_iters

    def _generate(self, N: int | None = None) -> np.ndarray:
        if N is not None:
            print("[Warning] N is ignored for truncated Gaussian noise, returning a single sample instead.")
        iters = 0
        w = super()._generate()
        while w not in self.trunc_bounds:
            w = super()._generate()
            iters += 1
            if iters > self.max_iters:
                raise Exception("exceeded max_iters of {0}, likely because of little overlap between the distribution and truncation polytope".format(self.max_iters))
        return w


class PolytopeVerticesNoise(NoiseBase):
    """Choses a random vertex of the vertices matrix as noise"""

    def __init__(self, W: Polytope, seed: int | None = None) -> None:
        assert seed is None or seed >= 0
        self.V = W.V
        self.rng = np.random.default_rng(seed)

    def _generate(self, N: int | None = None) -> np.ndarray:
        if N is None:
            idx = self.rng.choice(self.V.shape[0])
            sample = self.V[idx, :].reshape(-1, 1)
        else:
            idx = self.rng.choice(self.V.shape[0], N)
            sample = self.V[idx, :].T
        return sample


class PolytopeNoise(NoiseBase):
    """Samples a random disturbance vector within a polytope, where vertices are weighted uniformly"""

    def __init__(self, W: Polytope, seed: int | None = None) -> None:
        assert seed is None or seed >= 0
        self.V = W.V
        self.rng = np.random.default_rng(seed)

    def _generate(self, N: int | None = None) -> np.ndarray:
        """Based on implementation for randomPoint() in MPT"""
        if N is None:
            L = self.rng.uniform(size=(1, self.V.shape[0]))
            L /= np.sum(L)
            sample = (L @ self.V).reshape(-1, 1)
        else:
            L = self.rng.uniform(size=(N, self.V.shape[0]))
            L /= np.sum(L, axis=1).reshape(-1, 1)
            sample = (L @ self.V).T
        return sample


class StateDependentNoiseBase(NoiseBase):
    """Base class for state dependent random noise/disturbance generators"""

    state_dependent = True

    def generate(self, x: np.ndarray) -> np.ndarray:
        return self._generate(x)

    @classmethod
    @abstractmethod
    def _generate(self, x: np.ndarray) -> np.ndarray:
        """Noise generation method to be implemented by the inherited class"""
        raise NotImplementedError
    

class StateDependentNoise(StateDependentNoiseBase):
    """Generates state dependent noise based on a linear transformation G and a random uniform scalar in [0,1]"""

    def __init__(self, G: np.ndarray, seed: int | None = None) -> None:
        assert seed is None or seed >= 0
        self.G = G
        self.rng = np.random.default_rng(seed)

    def _generate(self, x: np.ndarray) -> np.ndarray:
        return (self.rng.uniform() * self.G @ x).reshape(-1,1)


"""
To test the noise generators, run this script directly. This will generate various noise samples and plot them.
"""
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    def generate_N_times(noise_generator, N=500, input=None):
        x = []
        y = []
        for i in range(N):
            if input is not None:
                w = noise_generator.generate(input)
            else:
                w = noise_generator.generate()
            x.append(w[0])
            y.append(w[1])
        return x, y

    """zero noise """
    print("testing zero noise...")
    zero_noise = ZeroNoise(dim=np.random.choice(10))
    assert np.all(zero_noise.generate(500) == 0.0)

    """gaussian noise """
    print("testing gaussian noise...")
    mean = np.array([0.5, 1.0]).reshape(-1, 1)
    covariance = np.diag([2.0, 0.5])
    gaussian_noise = GaussianNoise(mean, covariance)
    x = gaussian_noise.generate(500)

    plt.figure(1)
    plt.scatter(x[0], x[1])
    plt.grid()
    plt.title("gaussian noise")

    gaussian_noise.seed(42)
    x = gaussian_noise.generate(500)
    gaussian_noise.seed(42)
    xx = gaussian_noise.generate(500)
    assert np.allclose(x, xx)

    """truncated gaussian noise"""
    print("testing truncated gaussian noise...")
    mean = np.array([0.5, 1.0]).reshape(-1, 1)
    covariance = np.diag([2.0, 0.5])
    A_w = np.array([[1.0, 0.0], [0.0, 1.0], [-1.0, -0.0], [-0.0, -1.0]])
    b_w = np.array([2.0, 0.5, 0.0, 0.5])
    W = Polytope(A_w, b_w)
    trunc_gaussian_noise = TruncGaussianNoise(mean, covariance, W)
    x, y = generate_N_times(trunc_gaussian_noise)

    plt.figure(2)
    plt.scatter(x, y)
    W.plot(ax=plt.gca(), alpha=0.5)
    plt.grid()
    plt.title("truncated gaussian noise")

    trunc_gaussian_noise.seed(42)
    x, y = generate_N_times(trunc_gaussian_noise)
    trunc_gaussian_noise.seed(42)
    xx, yy = generate_N_times(trunc_gaussian_noise)
    assert np.allclose(x, xx) and np.allclose(y, yy)

    """generate a polytope"""
    x, y = generate_N_times(gaussian_noise, N=4)
    sampled_V = np.concatenate(
        [np.array(x).reshape(-1, 1), np.array(y).reshape(-1, 1)], axis=1
    )
    W = qhull(sampled_V)

    """polytope vertex noise"""
    print("testing polytope vertices noise...")
    vertices_noise = PolytopeVerticesNoise(W)
    x = vertices_noise.generate(100)

    plt.figure(3)
    plt.scatter(x[0], x[1])
    W.plot(ax=plt.gca(), alpha=0.5)
    plt.grid()
    plt.title("polytope vertices noise")

    vertices_noise.seed(42)
    x = vertices_noise.generate(100)
    vertices_noise.seed(42)
    xx = vertices_noise.generate(100)
    assert np.allclose(x, xx)

    """polytope random noise"""
    print("testing polytope noise...")
    polytope_noise = PolytopeNoise(W)
    x = polytope_noise.generate(500)

    plt.figure(4)
    plt.scatter(x[0], x[1])
    W.plot(ax=plt.gca(), alpha=0.5)
    plt.grid()
    plt.title("polytope uniform noise")

    polytope_noise.seed(42)
    x = polytope_noise.generate(100)
    polytope_noise.seed(42)
    xx = polytope_noise.generate(100)
    assert np.allclose(x, xx)


    """state dependent noise"""
    print("testing state dependent noise...")
    G = np.diag([0.0, 0.1])
    state_dependent_noise = StateDependentNoise(G)
    x, y = generate_N_times(state_dependent_noise, input=np.array([1.0, 2.0]).reshape(-1, 1))
    x_0 = G @ np.array([1.0, 2.0]).reshape(-1, 1)

    plt.figure(5)
    plt.scatter(x, y)
    plt.scatter(x_0[0], x_0[1], color="red", marker="x")
    plt.grid()
    plt.title("state dependent noise")

    state_dependent_noise.seed(42)
    x, y = generate_N_times(state_dependent_noise, input=np.array([1.0, 2.0]).reshape(-1, 1))
    state_dependent_noise.seed(42)
    xx, yy = generate_N_times(state_dependent_noise, input=np.array([1.0, 2.0]).reshape(-1, 1))
    assert np.allclose(x, xx) and np.allclose(y, yy)

    plt.show()
