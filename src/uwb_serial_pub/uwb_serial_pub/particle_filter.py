#!/usr/bin/env python3

import numpy as np
from typing import Sequence, Tuple


class ParticleFilter:
    """Particle filter tracking planar position, velocity, and heading."""

    def __init__(
        self,
        N: int,
        dt: float,
        sigma_pos: float,
        sigma_v: float,
        sigma_theta: float,
        cauchy_gamma: float,
        anchors: Sequence[Tuple[float, float]] = (
            (-0.225, 0.195),
            (0.225, 0.195),
            (-0.225, -0.195),
            (0.225, -0.195),
        ),
        neff_threshhold_ratio: float = 0.5,
    ):
        self.N = int(N)
        self.dt = float(dt)

        self.sigma_pos = float(sigma_pos)
        self.sigma_v = float(sigma_v)
        self.sigma_theta = float(sigma_theta)

        self.gamma = float(cauchy_gamma)
        self.anchors = np.asarray(anchors, dtype=np.float64)
        self.neff_threshhold_ratio = float(neff_threshhold_ratio)

        self.particles = np.zeros((self.N, 4), dtype=np.float64)
        self.weights = np.full(self.N, 1.0 / self.N, dtype=np.float64)

    def initialize(self, px_range, py_range, v_range, theta_range):
        self.particles[:, 0] = np.random.uniform(px_range[0], px_range[1], size=self.N)
        self.particles[:, 1] = np.random.uniform(py_range[0], py_range[1], size=self.N)
        self.particles[:, 2] = np.random.uniform(v_range[0], v_range[1], size=self.N)
        self.particles[:, 3] = np.random.uniform(theta_range[0], theta_range[1], size=self.N)

    def predict(self, dt: float | None = None):
        if dt is not None and dt > 0.0:
            self.dt = float(dt)
        step_dt = self.dt

        px = self.particles[:, 0]
        py = self.particles[:, 1]
        v = self.particles[:, 2]
        th = self.particles[:, 3]

        noise_px = np.random.normal(0.0, self.sigma_pos, self.N)
        noise_py = np.random.normal(0.0, self.sigma_pos, self.N)
        noise_v = np.random.normal(0.0, self.sigma_v, self.N)
        noise_th = np.random.normal(0.0, self.sigma_theta, self.N)

        px = px + v * np.cos(th) * step_dt + noise_px
        py = py + v * np.sin(th) * step_dt + noise_py
        v = v + noise_v
        th = th + noise_th

        th = (th + np.pi) % (2 * np.pi) - np.pi

        self.particles[:, 0] = px
        self.particles[:, 1] = py
        self.particles[:, 2] = v
        self.particles[:, 3] = th

    def update(self, ranges):
        ranges = np.asarray(ranges, dtype=float)
        if ranges.shape[0] != self.anchors.shape[0]:
            raise ValueError("Number of range measurements must match number of anchors")

        px = self.particles[:, 0]
        py = self.particles[:, 1]

        total_likelihood = np.ones(self.N)
        for i, (ax, ay) in enumerate(self.anchors):
            expected = np.sqrt((px - ax) ** 2 + (py - ay) ** 2)
            error = ranges[i] - expected
            likelihood = 1.0 / (1.0 + (error / self.gamma) ** 2)
            total_likelihood *= likelihood

        self.weights *= total_likelihood
        self.weights += 1e-300
        self.weights /= np.sum(self.weights)

    def neff(self):
        return 1.0 / np.sum(self.weights**2)

    def step(self, ranges, dt: float | None = None):
        self.predict(dt)
        self.update(ranges)
        if self.neff() < self.neff_threshhold_ratio * self.N:
            self.resample()

    def estimate(self):
        return np.average(self.particles, weights=self.weights, axis=0)

    def resample(self):
        positions = (np.arange(self.N) + np.random.uniform()) / self.N
        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1.0
        indexes = np.searchsorted(cumulative_sum, positions)

        self.particles = self.particles[indexes]
        self.weights.fill(1.0 / self.N)
