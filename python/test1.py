# pyright: reportUnknownMemberType=false, reportUnusedCallResult=false

from numpy.typing import NDArray
import numpy as np
Vector = NDArray[np.float64]
import matplotlib.pyplot as plt
#from typing import Callable

from boeing747_dynamics import Boeing747EulerDynamics, Boeing747RotationalDynamics, Boeing747TranslationalDynamics
import runge_kutta as rk
from typing import cast

# Aircraft inertia parameters
Ix = 0.247e8
Iy = 0.449e8
Iz = 0.673e8
Ixz = -0.212e7

m = 183500.0  # Mass of the aircraft (kg)
kappa = 0.5  # Placeholder for aerodynamic coefficient
T = 1.0  # Placeholder for thrust (N)

# Create aircraft model
aircraft_rot = Boeing747RotationalDynamics(Ix, Iy, Iz, Ixz)
aircraft_euler = Boeing747EulerDynamics()
aircraft_translational = Boeing747TranslationalDynamics(m, kappa, T)  # Placeholder for translational dynamics

# Initial conditions
t0 = 0.0 # Initial time
y0_rot = np.array([0.0, 0.0, 0.0])  # Initial angular rates (p, q, r)
y0_euler = np.array([0.0, 0.0, 0.0])  # Initial Euler angles (phi, theta, psi)
y0_translational = np.array([0.0, 0.0, 0.0])  # Initial translational state (U, V, W)

# Control inputs
u_rates = np.array([1000.0, 2000.0, 1500.0])
u_translational = np.array([5000.0, 0.0, 0.0])  # Placeholder for translational control inputs

# Evaluate differential equations

dy = aircraft_rot(t0, y0_rot, u_rates)

print("State derivatives:", dy)

# Use Runge-Kutta method to integrate the equations of motion

h = 0.01  # Time step
t_initial = 0.0  # Initial time
t_final = 1.0  # Final time
N = int((t_final - t_initial) / h)  # Number of time steps

# Vectors to store the results
# vetores da dinâmica rotacional
y_rot = np.zeros((N + 1, len(y0_rot)))  # Array to store state values
y_rot[0] = y0_rot
t_rot : list[float] = [float(t) for t in np.zeros(N + 1)]  # Array to store time values
t_rot[0] = t_initial

# vetores para a dinâmica de Euler
y_euler = np.zeros((N + 1, len(y0_euler)))  # Array to store Euler angles
y_euler[0] = y0_euler
t_euler : list[float] = [float(t) for t in np.zeros(N + 1)]  # Array to store time values
t_euler[0] = t_initial

# vetores para a dinâmica translacional
y_translational = np.zeros((N + 1, len(y0_translational)))  # Array to store translational state
y_translational[0] = y0_translational
t_translational : list[float] = [float(t) for t in np.zeros(N + 1)]  # Array to store time values
t_translational[0] = t_initial

# function handle for the aircraft rotational dynamics
f_rot: rk.Derivative = lambda t, y: aircraft_rot(t, y, u_rates)

# Integrate using Runge-Kutta method
for i in range(N):
    
    # Dinâmica rotacional
    t_rot[i + 1], y_rot[i + 1] = rk.rk4(
            f_rot, 
            t_rot[i], 
            cast(rk.State, y_rot[i]), 
            h
            )

    # Dinâmica de Euler
    f_euler: rk.Derivative = lambda t, y: aircraft_euler(t, y, cast(rk.State, y_rot[i+1]))

    t_euler[i + 1], y_euler[i + 1] = rk.rk4(
            f_euler,
            t_euler[i],
            cast(rk.State, y_euler[i]),
            h
            )

    # Dinâmica translacional
    f_translational: rk.Derivative = lambda t, y: aircraft_translational(t, y, cast(rk.State, y_euler[i+1]), cast(rk.State, y_rot[i+1]), u_translational)

    t_translational[i + 1], y_translational[i + 1] = rk.rk4(
            f_translational,
            t_translational[i],
            cast(rk.State, y_translational[i]),
            h
            )

# Print the results
for i in range(N + 1):
    print(f"Time: {t_rot[i]:.2f}, Rot State: {y_rot[i]}")
    print(f"Time: {t_euler[i]:.2f}, Euler Angles: {y_euler[i]}")
    print(f"Time: {t_translational[i]:.2f}, Translational State: {y_translational[i]}")

# plots
# Dinâmica rotacional
plt.figure(figsize=(10, 6))
plt.subplot(3, 1, 1)
plt.plot(t_rot, y_rot[:, 0], label='p (roll rate)')
plt.ylabel('p (rad/s)')
plt.grid()
plt.subplot(3, 1, 2)
plt.plot(t_rot, y_rot[:, 1], label='q (pitch rate)', color
='orange')
plt.ylabel('q (rad/s)')
plt.grid()
plt.subplot(3, 1, 3)
plt.plot(t_rot, y_rot[:, 2], label='r (yaw rate)', color
='green')
plt.xlabel('Time (s)')
plt.ylabel('r (rad/s)')
plt.grid()
plt.tight_layout()
# save the plot to disk
#plt.savefig('figures/rotational_dynamics.png')

# Dinâmica de Euler
plt.figure(figsize=(10, 6))
plt.subplot(3, 1, 1)
plt.plot(t_euler, y_euler[:, 0], label='phi (roll angle)')
plt.ylabel('phi (rad)')
plt.grid()
plt.subplot(3, 1, 2)
plt.plot(t_euler, y_euler[:, 1], label='theta (pitch angle)', color='orange')
plt.ylabel('theta (rad)')
plt.grid()
plt.subplot(3, 1, 3)
plt.plot(t_euler, y_euler[:, 2], label='psi (yaw angle)', color='green')
plt.xlabel('Time (s)')
plt.ylabel('psi (rad)')
plt.grid()
plt.tight_layout()
# save the plot to disk
#plt.savefig('figures/euler_dynamics.png')

# Dinâmica translacional
plt.figure(figsize=(10, 6))
plt.subplot(3, 1, 1)
plt.plot(t_translational, y_translational[:, 0], label='U (forward velocity)')
plt.ylabel('U (m/s)')
plt.grid()
plt.subplot(3, 1, 2)
plt.plot(t_translational, y_translational[:, 1], label='V (side velocity)', color='orange')
plt.ylabel('V (m/s)')
plt.grid()
plt.subplot(3, 1, 3)
plt.plot(t_translational, y_translational[:, 2], label='W (vertical velocity)', color='green')
plt.xlabel('Time (s)')
plt.ylabel('W (m/s)')
plt.grid()
plt.tight_layout()
# save the plot to disk
#plt.savefig('figures/translational_dynamics.png')
plt.show()
