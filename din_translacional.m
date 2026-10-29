% =========================================================================
% Equaçao diferencia do controle Lateral e Longitudinal de um Boeing 747
% Dinamica Translacional
% -------------------------------------------------------------------------
% Autor: Diego Ferruzzo
% Data: 28/10/2026
% Ref.: Sistemas de Controle para Engenharia. Franklin, Gene F. 6a ed. 
%       2013. Pag. 593.
% =========================================================================
function dy = din_translacional(t,y,y_rot,y_euler,u,params)
% -------------------------------------------------------------------------
% t: tempo;
% y: vetor de estados, y = [U,V,W];
% y_rot : vetor de estados dinamica rotacional nas taxas, y_rot = [p,q,r];
% y_euler : angulos de Euler, y_euler = [phi,theta,psi]; 
% u : vetor de entrada, u = [X,Y,Z];
% params : vetor de parametros, params = [g,m,kappa,T];
% -------------------------------------------------------------------------
% Vetor de estados:
U = y(1);
V = y(2);
W = y(3);
% Vetor de estados da dinamica rotacional
p = y_rot(1);
q = y_rot(2);
r = y_rot(3);
% Angulos de Euler
phi = y_euler(1);
theta = y_euler(2);
psi = y_euler(3);
% Vetor de entrada
X = u(1);
Y = u(2);
Z = u(3);
% Parametros
g = params(1);
m = params(2);
kappa = params(3);
T = params(4);
% -------------------
% Equaçao diferencial
% -------------------
% dU/dt:
dy(1) = -g*sin(theta) + (1/m)*kappa*T*cos(theta) - q*W + r*V + X/m;
% dV/dt:
dy(2) = g*cos(theta)*sin(phi) - r*U + p*W + Y/m;
% dW/dt:
dy(3) = g*cos(theta)*cos(phi) - (1/m)*kappa*T*sin(theta) - p*V + q*U + Z/m;
end