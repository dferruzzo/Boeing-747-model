% =========================================================================
% Teste da dinamica rotacional (p,q,r)
% Autor: Diego Ferruzzo
% Data: 28/10/2026
% =========================================================================
clear;
close all;
clc;
% Vetor de entrada
L = 1.0;
M = 0;
N = 0;
u = [L,M,N];
% parametros
Ix = 1e-3;
Iy = 2e-3;
Iz = 3e-3;
Ixz = 1e-3;
params = [Ix,Iy,Iz,Ixz];
%% Dinamica sem controle
% equaçao diferencial:
f = @(t,y) din_rotacional(t,y,u,params); 
% parametros da simulaçao
t0 = 0;
tf = 1;
h = 1e-3;
y0 = [0.1,0.3,0.5];
%
N = floor(tf-t0)/h;
t = nan(N,1);
y = nan(N,3);
t(1) = t0;
y(1,:) = y0;
%
for i = 1:N-1
    [t(i+1), y(i+1,:)] = rk4(f, t(i), y(i,:), h);
end
%% figuras
figure(1);
plot(t,y(:,1),'-b');
hold on;
plot(t,y(:,2),'-r');
plot(t,y(:,3),'-g');
legend('p(t)','q(t)','r(t)');
grid on;
%% Dinamica com controle
