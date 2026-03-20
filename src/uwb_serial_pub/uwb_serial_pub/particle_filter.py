#!/usr/bin/env python3

import numpy as np
from typing import Sequence, Tuple


class ParticleFilter:
    """
    Particle filter for planar localization.

    State vector is ordered as:
      x_k = [x, y, theta, v]^T
    where (x,y) is 2D position, theta is heading, and v is velocity.
    """

    def __init__(
        self,
        N: int,
        dt: float,
        sigma: float,
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

        # Measurement likelihood std [m].
        self.sigma = float(sigma)

        self.anchors = np.asarray(anchors, dtype=np.float64)
        self.neff_threshhold_ratio = float(neff_threshhold_ratio)

        # Particles: [x, y, theta, v]
        self.particles = np.zeros((self.N, 4), dtype=np.float64)
        self.weights = np.full(self.N, 1.0 / self.N, dtype=np.float64)

    def initialize(self, x_range, y_range, theta_range, v_range):
        self.particles[:, 0] = np.random.uniform(x_range[0], x_range[1], size=self.N)
        self.particles[:, 1] = np.random.uniform(y_range[0], y_range[1], size=self.N)
        self.particles[:, 2] = np.random.uniform(
            theta_range[0], theta_range[1], size=self.N
        )
        self.particles[:, 3] = np.random.uniform(v_range[0], v_range[1], size=self.N)

    def predict(self, dt: float | None = None):
        if dt is not None and dt > 0.0:
            self.dt = float(dt)
        step_dt = self.dt

        # Unpack state
        x = self.particles[:, 0]
        y = self.particles[:, 1]
        theta = self.particles[:, 2]
        v = self.particles[:, 3]

        # Non-linear motion model from the spec:
        #   x_k = x_{k-1} + v_{k-1} cos(theta_{k-1}) dt
        #   y_k = y_{k-1} + v_{k-1} sin(theta_{k-1}) dt
        x_new = x + v * np.cos(theta) * step_dt
        y_new = y + v * np.sin(theta) * step_dt

        # Spec does not define dynamics for theta/v, so we keep them constant.
        theta_new = theta

        theta_new = (theta_new + np.pi) % (2.0 * np.pi) - np.pi

        self.particles[:, 0] = x_new
        self.particles[:, 1] = y_new
        self.particles[:, 2] = theta_new
        self.particles[:, 3] = v

    def update(self, ranges):
        ranges = np.asarray(ranges, dtype=float)
        if ranges.shape[0] != self.anchors.shape[0]:
            raise ValueError("Number of range measurements must match number of anchors")

        x = self.particles[:, 0]
        y = self.particles[:, 1]

        # Likelihood: Gaussian on Euclidean distance errors.
        #   h^(i)(x_k) = sqrt((x_k-x_i)^2 + (y_k-y_i)^2)
        #   p(z_i | x_k) ∝ exp( -0.5 * ((z_i - h^(i)(x_k))/sigma)^2 )
        # We do everything in log-space for numerical stability.
        eps = 1e-300
        log_w = np.log(self.weights + eps)
        log_total_likelihood = np.zeros(self.N, dtype=np.float64)

        for i, (ax, ay) in enumerate(self.anchors):
            expected = np.sqrt((x - ax) ** 2 + (y - ay) ** 2)
            error = ranges[i] - expected
            log_total_likelihood += -0.5 * (error / self.sigma) ** 2

        log_w = log_w + log_total_likelihood
        # Normalize weights stably: w_i ∝ exp(log_w_i - max(log_w))
        max_log_w = float(np.max(log_w))
        w_unnorm = np.exp(log_w - max_log_w)
        w_sum = float(np.sum(w_unnorm))
        if w_sum <= 0.0 or not np.isfinite(w_sum):
            self.weights.fill(1.0 / self.N)
        else:
            self.weights = w_unnorm / w_sum

    def neff(self):
        return 1.0 / np.sum(self.weights**2)

    def step(self, ranges, dt: float | None = None):
        self.predict(dt)
        self.update(ranges)
        # Systematic resampling when N_eff < N/2 (by default).
        if self.neff() < self.neff_threshhold_ratio * self.N:
            self.resample()

    def estimate(self):
        # Returns [x, y, theta, v].
        # Note: theta is circular, so we compute a circular weighted mean.
        w = self.weights
        x_mean = float(np.sum(w * self.particles[:, 0]))
        y_mean = float(np.sum(w * self.particles[:, 1]))

        theta = self.particles[:, 2]
        theta_mean = float(
            np.arctan2(np.sum(w * np.sin(theta)), np.sum(w * np.cos(theta)))
        )

        v_mean = float(np.sum(w * self.particles[:, 3]))
        return np.array([x_mean, y_mean, theta_mean, v_mean], dtype=np.float64)

    def resample(self):
        positions = (np.arange(self.N) + np.random.uniform()) / self.N
        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1.0
        indexes = np.searchsorted(cumulative_sum, positions)

        self.particles = self.particles[indexes]
        self.weights.fill(1.0 / self.N)
