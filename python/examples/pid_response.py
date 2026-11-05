from boeing747.controllers import PidDiscreteStandardFiltered
from runge_kutta import State, rk4, Derivative
import numpy as np
from numpy.typing import NDArray
from typing import cast
import matplotlib.pyplot as plt

"""
Teste do controlador PID discreto com filtro derivativo.
"""

Vector = NDArray[np.float64]

# Definição da função a integrar (sistema de primeira ordem)
def primeira_ordem(y: State, u: float):
    """
    Teste do controlador PID discreto com filtro derivativo em um sistema de primeira ordem.
    """
    
    # Sistema de primeira ordem: dy/dt = -a*y + u
    a = 1.0
    return -y + u
    
# Parâmetros da simulação
t0 = 0.0
tf = 10.0
dt = 0.1

y0 = 0.1  # Condição inicial

n = int((tf - t0) / dt)
y = np.zeros(n + 1)
y[0] = y0
t = np.zeros(n + 1)
t[0] = t0

y_malha_aberta = np.zeros(n + 1)
y_malha_aberta[0] = y0

y_unit_feedback = np.zeros(n + 1)
y_unit_feedback[0] = y0

pid = PidDiscreteStandardFiltered(Kp=1.0, Ki=0.1, Kd=0.01, N=10, Ts=0.1)
# Vetores para armazenar histórico de erros e saídas do controlador
error_history: Vector = np.zeros(3)
u_history: Vector = np.zeros(2)

# Setpoint desejado
setpoint = 1.0

# integração utilizando RK4
for i in range(n):
    # Erro de controle
    error = setpoint - y[i]
    
    error_history = np.concatenate(([error], error_history[:-1]))
    u = pid(error_history, u_history)
    u_history = np.concatenate(([u], u_history[:-1]))

    f : Derivative = lambda t, y: primeira_ordem(y, u)

    t[i + 1], y[i + 1] = rk4(f, t[i], y[i], dt)

    # integração do sistema sem controlador
    f_malha_aberta : Derivative = lambda t, y: primeira_ordem(y, 0.0)
    __, y_malha_aberta[i + 1] = rk4(f_malha_aberta, t[i], y_malha_aberta[i], dt)

    # integração do sistema com realimentação unitária
    f_unit_feedback : Derivative = lambda t, y: primeira_ordem(y, setpoint - y_unit_feedback[i])
    __, y_unit_feedback[i + 1] = rk4(f_unit_feedback, t[i], y_unit_feedback[i], dt)
    
print("State values:", y)

# graficar os resultados
plt.figure()
plt.plot(t, y, label='Saída do sistema com controlador PID discreto')
plt.plot(t, y_malha_aberta, label='Saída do sistema em malha aberta')
plt.plot(t, y_unit_feedback, label='Saída do sistema com realimentação unitária')
plt.axhline(y=setpoint, color='r', linestyle='--', label='Setpoint')
plt.xlabel('Tempo (s)')
plt.ylabel('Saída')
plt.legend()
plt.title('Resposta do sistema de primeira ordem com controlador PID discreto')
plt.legend()
plt.grid()
plt.show()
