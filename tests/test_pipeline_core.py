"""Unit tests for the pure numerical parts of the pipeline.

Run from the project root:
    pytest -q
"""
import os
import sys

import numpy as np
import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "experiments"))

import evaluate_canonical as ev  # noqa: E402
from core.matcher import MatcherError, validate_lightglue_conf  # noqa: E402


def random_rotation(rng):
    q, r = np.linalg.qr(rng.normal(size=(3, 3)))
    q = q * np.sign(np.diag(r))
    if np.linalg.det(q) < 0:
        q[:, 0] *= -1
    return q


def test_umeyama_recovers_known_similarity():
    rng = np.random.default_rng(0)
    src = rng.normal(size=(60, 3))
    R = random_rotation(rng)
    s, t = 2.5, np.array([1.0, -2.0, 3.0])
    dst = (s * (R @ src.T)).T + t

    s_hat, R_hat, t_hat = ev.umeyama(src, dst)

    assert np.isclose(s_hat, s)
    assert np.allclose(R_hat, R, atol=1e-8)
    assert np.allclose(t_hat, t, atol=1e-8)


def test_integrate_straight_line_accumulates_unit_steps():
    steps = [{"valid": True, "u": [1.0, 0.0], "R": np.eye(3), "scale": 1.0} for _ in range(5)]

    traj = ev.integrate(steps)

    assert np.allclose(traj[-1], [5.0, 0.0, 0.0])


def test_integrate_skips_invalid_steps():
    steps = [
        {"valid": True, "u": [1.0, 0.0], "R": np.eye(3), "scale": 1.0},
        {"valid": False, "u": [1.0, 0.0], "R": np.eye(3), "scale": 1.0},
    ]

    traj = ev.integrate(steps)

    assert np.allclose(traj[-1], [1.0, 0.0, 0.0])


def test_integrate_keeps_z_at_zero():
    rng = np.random.default_rng(1)
    steps = [
        {"valid": True, "u": rng.normal(size=2), "R": random_rotation(rng), "scale": 1.0}
        for _ in range(20)
    ]

    traj = ev.integrate(steps)

    assert np.allclose(traj[:, 2], 0.0)


def test_lightglue_rejects_width_pruning_disabled():
    with pytest.raises(MatcherError):
        validate_lightglue_conf(depth_confidence=0.95, width_confidence=-1.0)


def test_lightglue_rejects_aggressive_depth_exit():
    with pytest.raises(MatcherError):
        validate_lightglue_conf(depth_confidence=0.99, width_confidence=0.99)


def test_lightglue_accepts_default_settings():
    validate_lightglue_conf(depth_confidence=0.95, width_confidence=0.99)
