% =========================================================================
% Equaçao diferencia do controle Lateral e Longitudinal de um Boeing 747
% Dinamica Rotacional nas taxas
% -------------------------------------------------------------------------
% Autor: Diego Ferruzzo
% Data: 28/10/2026
% Ref.: Sistemas de Controle para Engenharia. Franklin, Gene F. 6a ed. 
%       2013. Pag. 593.
% =========================================================================
function dy = din_rotacional(t,y,u,params)
% ------------------------------------------------------
% t eh o tempo
% y eh o vetor de estados y= [p,q,r]
% u eh o vetor de entrada u = [L,M,N]
% params e o vetor de parametros params = [Ix,Iy,Iz,Ixz]
% ------------------------------------------------------
% 
% Vetor de estados
p = y(1);
q = y(2);
r = y(3);
% vetor de entrada
L = u(1);
M = u(2);
N = u(3);
% parametros
Ix = params(1);
Iy = params(2);
Iz = params(3);
Ixz = params(4);
% -------------------
% Equaçao diferencial
% -------------------
% dp/dt:
dy(1) = (1/(Iz*Ix-power(Ixz,2)))*( ...
    + Iz*L ...
    - Ixz*N ...
    - (Iz*(Iz-Iy)+power(Ixz,2))*q*r ...
    - Ixz*(Iz - Iy + Ix)*q*p ...
    );
% dq/dt:
dy(2) = M/Iy ...
    - ((Ix - Iz)/Iy)*p*r ...
    -(Ixz/Iy)*(power(r,2) - power(p,2));
% dr/dr:
dy(3) = (1/(power(Ixz,2) - Ix*Iz))*(...
    + Ixz*L ...
    - Ix*N ...
    - ( Ixz*(Iz - Iy) + Ix*Ixz )*q*r ...
    - ( power(Ixz,2) - Ix*(Iy - Ix) )*q*p ...
    );
end