import numpy as np

from boeing747 import (
    Boeing747EulerDynamics,
    Boeing747RotationalDynamics,
    Boeing747TranslationalDynamics,
)


def test_rotational_dynamics_at_rest_has_no_acceleration() -> None:
    dynamics = Boeing747RotationalDynamics(
        Ix=0.247e8,
        Iy=0.449e8,
        Iz=0.673e8,
        Ixz=-0.212e7,
    )

    derivative = dynamics(0.0, np.zeros(3), np.zeros(3))

    np.testing.assert_allclose(derivative, np.zeros(3))


def test_euler_dynamics_matches_rates_at_level_attitude() -> None:
    dynamics = Boeing747EulerDynamics()
    rates = np.array([0.1, -0.2, 0.3])

    derivative = dynamics(0.0, np.zeros(3), rates)

    np.testing.assert_allclose(derivative, rates)


def test_translational_dynamics_at_rest_includes_gravity_and_forces() -> None:
    dynamics = Boeing747TranslationalDynamics(m=100.0, kappa=0.5, T=20.0)

    derivative = dynamics(
        0.0,
        np.zeros(3),
        np.zeros(3),
        np.zeros(3),
        np.array([5.0, 10.0, -20.0]),
    )

    np.testing.assert_allclose(derivative, np.array([0.15, 0.1, 9.61]))
