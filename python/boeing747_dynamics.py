from numpy.typing import NDArray
import numpy as np
from typing import cast
from runge_kutta import State

Vector = NDArray[np.float64]

class Boeing747RotationalDynamics:
    """
    Rotational dynamics of a Boeing 747 aircraft.

    State vector:
        y = [p, q, r]

    Input vector:
        u = [L, M, N]

    Parameters:
        Ix, Iy, Iz, Ixz: Aircraft moments and product of inertia.

    Reference:
        Franklin, Gene F. et al.
        Sistemas de Controle para Engenharia, 6th ed., 2013, p. 593.
    """

    def __init__(self, Ix: float, Iy: float, Iz : float, Ixz: float):

        self.Ix : float = Ix
        self.Iy : float = Iy
        self.Iz : float= Iz
        self.Ixz : float = Ixz

    def __call__(self, t: float, y: State, u: State) -> State:
        """
        Computes the rotational dynamics.

        Parameters
        ----------
        t : float
            Time (s).
        y : array_like, shape (3,)
            State vector [p, q, r].
        u : array_like, shape (3,)
            Input vector [L, M, N].

        Returns
        -------
        dy : ndarray, shape (3,)
            State derivatives [dp/dt, dq/dt, dr/dt].
        """

        state = np.asarray(y, dtype=np.float64)

        p = cast(np.float64, state[0])
        q = cast(np.float64, state[1])
        r = cast(np.float64, state[2])
        
        u_state = np.asarray(u, dtype=np.float64)
        L = cast(np.float64, u_state[0])
        M = cast(np.float64, u_state[1]) 
        N = cast(np.float64, u_state[2])

        Ix = self.Ix
        Iy = self.Iy
        Iz = self.Iz
        Ixz = self.Ixz

        # --------------------------------------------------
        # Differential equations
        # --------------------------------------------------

        dp = (1.0 / (Iz * Ix - Ixz**2)) * (
            Iz * L
            - Ixz * N
            - (Iz * (Iz - Iy) + Ixz**2) * q * r
            - Ixz * (Iz - Iy + Ix) * q * p
        )

        dq = (
            M / Iy
            - ((Ix - Iz) / Iy) * p * r
            - (Ixz / Iy) * (r**2 - p**2)
        )

        dr = (1.0 / (Ixz**2 - Ix * Iz)) * (
            Ixz * L
            - Ix * N
            - (Ixz * (Iz - Iy) + Ix * Ixz) * q * r
            - (Ixz**2 - Ix * (Iy - Ix)) * q * p
        )

        return np.array([dp, dq, dr])

class Boeing747EulerDynamics:
    """
    Euler-angle kinematics of a Boeing 747 aircraft.

    State vector:
        y = [phi, theta, psi]

    Rotational rates:
        y_rot = [p, q, r]

    Reference:
        Bryson, A. E.
        Control of Spacecraft and Aircraft, 1994.
    """

    def __call__(self, t: float, y: State, y_rot: State)-> State:
        """
        Computes the Euler-angle derivatives.

        Parameters
        ----------
        t : float
            Time (s).
        y : array_like, shape (3,)
            Euler angles [phi, theta, psi] (rad).
        y_rot : array_like, shape (3,)
            Rotational rates [p, q, r] (rad/s).

        Returns
        -------
        dy : ndarray, shape (3,)
            Euler-angle derivatives [dphi, dtheta, dpsi].
        """
        y_state = np.asarray(y, dtype=np.float64)

        phi : float = cast(np.float64, y_state[0])
        theta : float = cast(np.float64, y_state[1])
        #psi : float = cast(np.float64, y_state[2])
        
        y_rot_state = np.asarray(y_rot, dtype=np.float64)
        
        p = cast(np.float64, y_rot_state[0])
        q = cast(np.float64, y_rot_state[1])
        r = cast(np.float64, y_rot_state[2])

        # --------------------------------------------------
        # Euler-angle kinematics
        # --------------------------------------------------

        dphi : float = cast(np.float64, (
            p
            + q * np.sin(phi) * np.tan(theta)
            + r * np.cos(phi) * np.tan(theta)
        ))

        dtheta : float = cast(np.float64, (
            q * np.cos(phi)
            + r * np.sin(phi)
        ))

        dpsi : float = cast(np.float64, (
            q * np.sin(phi) / np.cos(theta)
            + r * np.cos(phi) / np.cos(theta)
        ))

        return np.array([dphi, dtheta, dpsi])
