% =========================================================================
% PID Discreto com filtro
% Autor: Diego Ferruzzo
% Data: 28/10/2026
% Ref.: https://www.scilab.org/discrete-time-pid-controller-implementation
% =========================================================================
function u = pid_filtrado(Kp, Ki, Kd, N, Ts, e, u_hist)
% ------------------------------------------------------------------------- 
% Kp: Ganho proporcional
% Ki: Ganho integral
% Kd: Ganho derivativo
% N: Fator de filtragem do termo derivativo
% Ts: Período de amostragem
% e: Erro de controle (setpoint - saída), e = [e(k), e(k-1), e(k-2)]
% u_hist : u anteriores, u_hist = [u(k-1), u(k-2)]
%
% Como utilizar:
%
% e_hist = zeros(3,1);
% h_hist = zeros(2,1);
%
% loop de controle:
%   ...
%   e_hist = [erro; e_hist(1:end-1)];
%   u = pid_filtrado(Kp, Ki, Kd, N, Ts, e_hist, u_hist)
%   u_hist = [u; u_hist(1:end-1)];
%   ...
% end
% -------------------------------------------------------------------------
% Calculo dos parâmetros do controlador discreto com filtro
b0 = Kp*(1 + N*Ts) + Ki*Ts*(1 + N* N*Ts) + Kd*N;
b1 = -( Kp*( 2 + N*Ts ) + Ki*Ts + 2*Kd*N );
b2 = Kp + Kd*N;
a0 = 1 + N*Ts;
a1 = -( 2 + N*Ts );
a2 = 1;
% -------------------------------------------------------------------------
%
ek  = e(1); % e(k)
ek1 = e(2); % e(k-1);
ek2 = e(3); % e(k-2);
uk1 = u_hist(1); % u(k-1);
uk2 = u_hist(2); % u(k-2);
%
% O controlador
u = -(a1/a0)*uk1 - (a2/a0)*uk2 + (b0/a0)*ek + (b1/a0)*ek1 + (b2/a0)*ek2;
end
