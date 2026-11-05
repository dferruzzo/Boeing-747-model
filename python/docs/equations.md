# Model conventions

`boeing747_dynamics.py` contains all aircraft dynamics classes:

- `Boeing747RotationalDynamics` models body angular rates `[p, q, r]` from moments `[L, M, N]`.
- `Boeing747EulerDynamics` maps body angular rates to Euler-angle derivatives `[phi, theta, psi]`.
- `Boeing747TranslationalDynamics` models body velocities `[U, V, W]` from forces `[X, Y, Z]`.

The inputs and states use SI units: seconds, kilograms, meters, radians, newtons,
and newton-meters. Euler-angle kinematics are singular when `theta` approaches
plus or minus 90 degrees.
