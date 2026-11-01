# pyright: reportUnknownMemberType=false, reportUnusedCallResult=false

from numpy.typing import NDArray
import numpy as np
Vector = NDArray[np.float64]
import matplotlib.pyplot as plt
#from typing import Callable

from boeing747_dynamics import Boeing747EulerDynamics, Boeing747RotationalDynamics
import runge_kutta as rk
from typing import cast

# Aircraft inertia parameters
Ix = 0.247e8
Iy = 0.449e8
Iz = 0.673e8
Ixz = -0.212e7

# Create aircraft model
aircraft_rot = Boeing747RotationalDynamics(Ix, Iy, Iz, Ixz)
aircraft_euler = Boeing747EulerDynamics()

# Initial conditions
y0_rot = np.array([0.0, 0.0, 0.0])  # Initial angular rates (p, q, r)
y0_euler = np.array([0.0, 0.0, 0.0])  # Initial Euler angles (phi, theta, psi)

# Control inputs
u = np.array([1000.0, 2000.0, 1500.0])

# Evaluate differential equations
t = 0.0
dy = aircraft_rot(t, y0_rot, u)

print("State derivatives:", dy)

# Use Runge-Kutta method to integrate the equations of motion

# function handle for the aircraft dynamics
f_rot: rk.Derivative = lambda t, y: aircraft_rot(t, y, u)

#
h = 0.01  # Time step
t_initial = 0.0  # Initial time
t_final = 1.0  # Final time
N = int((t_final - t_initial) / h)  # Number of time steps

# Vectors to store the results
# vetores da dinâmica rotacional
y_values = np.zeros((N + 1, len(y0_rot)))  # Array to store state values
y_values[0] = y0_rot
t_values : list[float] = [float(t) for t in np.zeros(N + 1)]  # Array to store time values
t_values[0] = t_initial

# vetores para a dinâmica de Euler
y_euler = np.zeros((N + 1, len(y0_euler)))  # Array to store Euler angles
y_euler[0] = y0_euler
t_values_euler : list[float] = [float(t) for t in np.zeros(N + 1)]  # Array to store time values
t_values_euler[0] = t_initial

# Integrate using Runge-Kutta method
for i in range(N):
    t_values[i + 1], y_values[i + 1] = rk.rk4(
            f_rot, 
            t_values[i], 
            cast(rk.State, y_values[i]), 
            h
            )

    f_euler: rk.Derivative = lambda t, y: aircraft_euler(t, y, cast(rk.State, y_values[i+1]))

    t_values_euler[i + 1], y_euler[i + 1] = rk.rk4(
            f_euler,
            t_values_euler[i],
            cast(rk.State, y_euler[i]),
            h
            )

# Print the results
for i in range(N + 1):
    print(f"Time: {t_values[i]:.2f}, Rot State: {y_values[i]}")
    print(f"Time: {t_values_euler[i]:.2f}, Euler Angles: {y_euler[i]}")

# plots

plt.figure(figsize=(10, 6))
plt.subplot(3, 1, 1)
plt.plot(t_values, y_values[:, 0], label='p (roll rate)')
plt.ylabel('p (rad/s)')
plt.grid()
plt.subplot(3, 1, 2)
plt.plot(t_values, y_values[:, 1], label='q (pitch rate)', color
='orange')
plt.ylabel('q (rad/s)')
plt.grid()
plt.subplot(3, 1, 3)
plt.plot(t_values, y_values[:, 2], label='r (yaw rate)', color
='green')
plt.xlabel('Time (s)')
plt.ylabel('r (rad/s)')
plt.grid()
plt.tight_layout()
# save the plot to disk
plt.savefig('rotational_dynamics.png')

plt.figure(figsize=(10, 6))
plt.subplot(3, 1, 1)
plt.plot(t_values_euler, y_euler[:, 0], label='phi (roll angle)')
plt.ylabel('phi (rad)')
plt.grid()
plt.subplot(3, 1, 2)
plt.plot(t_values_euler, y_euler[:, 1], label='theta (pitch angle)', color='orange')
plt.ylabel('theta (rad)')
plt.grid()
plt.subplot(3, 1, 3)
plt.plot(t_values_euler, y_euler[:, 2], label='psi (yaw angle)', color='green')
plt.xlabel('Time (s)')
plt.ylabel('psi (rad)')
plt.grid()
plt.tight_layout()
# save the plot to disk
plt.savefig('euler_dynamics.png')


