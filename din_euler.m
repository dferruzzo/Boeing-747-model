% Equaçao diferencia do controle Lateral e Longitudinal de um Boeing 747
% Dinamica dos angulos de Euler
% -------------------------------------------------------------------------
% Autor: Diego Ferruzzo
% Data: 28/10/2026
% Ref.: Control of spacecraft and aircraft.  Bryson, 1994.
% =========================================================================
function dy = din_euler(t,y,y_rot)
% -------------------------------------------------------------------------
% t : tempo;
% y : vetor de estados, angulos de Euler, y = [phi,theta,psi];
% y_rot : vetor de estados da din. rotacional y_rot = [p,q,r];
% -------------------------------------------------------------------------
%
% Vetor de estados
phi = y(1);
theta = y(2);
psi = y(3);
% Vetor das taxas
p = y_rot(1);
q = y_rot(2);
r = y_rot(3);
% -------------------
% Equaçao diferencial
% -------------------
% dphi/dt :
dy(1) = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
% theta/dt :
dy(2) = q*cos(phi) + r*sin(phi);
% dpsi/dt :
dy(3) = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);
end