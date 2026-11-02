from runge_kutta import State
import numpy as np
from numpy.typing import NDArray

Vector = NDArray[np.float64]

class PidDiscreteStandardFiltered:
    """
    
    Kp: Ganho proporcional
    Ki: Ganho integral
    Kd: Ganho derivativo
    N: Fator de filtragem do termo derivativo
    Ts: Período de amostragem
    e: Erro de controle (setpoint - saída), e = [e(k), e(k-1), e(k-2)]
    u_hist : u anteriores, u_hist = [u(k-1), u(k-2)]

    Como utilizar:

    pid = PidDiscreteStandardFiltered(Kp, Ki, Kd, N, Ts)
    error_history = numpy.zeros((3,1));
    h_history = numpy.zeros((2,1));

    loop de controle:

       ...
       error_history = numpy.vstack(([[erro]], e_hist[:-1]));
       u = pid(error_history, u_history)
       u_history = numpy.vstack(([[u]], u_history[:-1]));
       ...

    end

    """
    def __init__(self, Kp: float, Ki: float, Kd:float, N: int, Ts: float):
        self.Kp : float  = Kp
        self.Ki : float = Ki
        self.Kd : float = Kd
        self.integral = 0
        self.previous_error = 0
        # Calculo dos parâmetros do PID discreto com filtro derivativo
        self.b0 : float = Kp*(1 + N*Ts) + Ki*Ts*(1 + N* N*Ts) + Kd*N;
        self.b1 : float = -( Kp*( 2 + N*Ts ) + Ki*Ts + 2*Kd*N );
        self.b2 : float = Kp + Kd*N;
        self.a0 : float = 1 + N*Ts;
        self.a1 : float = -( 2 + N*Ts );
        self.a2 : float = 1;

    def __call__(self, error_history: Vector, u_history: Vector)-> float:
        a0 : float = self.a0
        a1 : float = self.a1
        a2 : float = self.a2
        b0 : float = self.b0
        b1 : float = self.b1
        b2 : float = self.b2

        ek  : float = error_history[0] # e(k)
        ek1 : float = error_history[1] # e(k-1);
        ek2 : float = error_history[2] # e(k-2);
        uk1 : float = u_history[0]     # u(k-1);
        uk2 : float = u_history[1]     # u(k-2);

        u: float = -(a1/a0)*uk1 - (a2/a0)*uk2 + (b0/a0)*ek + (b1/a0)*ek1 + (b2/a0)*ek2;

        return u
