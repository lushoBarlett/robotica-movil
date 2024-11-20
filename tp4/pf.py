""" Written by Brian Hou for CSE571: Probabilistic Robotics (Winter 2019)
"""

import numpy as np

from utils import minimized_angle


class ParticleFilter:
    def __init__(self, mean, cov, num_particles, alphas, beta):
        self.alphas = alphas
        self.beta = beta

        self._init_mean = mean
        self._init_cov = cov
        self.num_particles = num_particles
        self.reset()

    def reset(self):
        self.particles = np.zeros((self.num_particles, 3))
        for i in range(self.num_particles):
            self.particles[i, :] = np.random.multivariate_normal(
                self._init_mean.ravel(), self._init_cov)
        self.weights = np.ones(self.num_particles) / self.num_particles

    def update(self, env, u, z, marker_id):
        """Update the state estimate after taking an action and receiving a landmark
        observation.

        u: action
        z: landmark observation
        marker_id: landmark ID
        """
        particles = np.zeros((self.num_particles, 3))
        for i in range(self.num_particles):
            # move the particles a bit
            particles[i, 0] = self.particles[i, 0] + np.random.normal(0, 2)
            particles[i, 1] = self.particles[i, 1] + np.random.normal(0, 2)
            particles[i, 2] = self.particles[i, 2] + np.random.normal(0, 0.1)
            particles[i, :] = np.random.multivariate_normal( # add noise to the motion model
                env.forward(particles[i, :], u).ravel(), # motion model
                env.noise_from_motion(u, self.alphas) # motion covariance
            )
        self.particles = particles

        for i in range(self.num_particles):
            self.weights[i] = env.likelihood(minimized_angle(env.observe(self.particles[i, :], marker_id) - z), self.beta)

        # normalize
        self.weights /= np.sum(self.weights)

        self.particles, self.weights = self.resample(self.particles, self.weights)

        return self.mean_and_variance(self.particles)

    def resample(self, particles, weights):
        """Sample new particles and weights given current particles and weights. Be sure
        to use the low-variance sampler from class.

        particles: (n x 3) matrix of poses
        weights: (n,) array of weights
        """
        new_particles = np.zeros((self.num_particles, 3))
        J = self.num_particles
        r = np.random.uniform(0, 1/J)
        c = weights[0]
        i = 0
        for m in range(J):
            U = r + m / J
            while U > c:
                i += 1
                c += weights[i]
            new_particles[m, :] = particles[i, :]

        return new_particles, [1 / J] * J

    def mean_and_variance(self, particles):
        """Compute the mean and covariance matrix for a set of equally-weighted
        particles.

        particles: (n x 3) matrix of poses
        """
        mean = particles.mean(axis=0)
        mean[2] = np.arctan2(
            np.cos(particles[:, 2]).sum(),
            np.sin(particles[:, 2]).sum()
        )

        zero_mean = particles - mean
        for i in range(zero_mean.shape[0]):
            zero_mean[i, 2] = minimized_angle(zero_mean[i, 2])
        cov = np.dot(zero_mean.T, zero_mean) / self.num_particles

        return mean.reshape((-1, 1)), cov
