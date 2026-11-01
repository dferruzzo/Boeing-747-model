% =========================================================================
% Exemplo de aplicaçao do pid_filtrado.m
% Autor: Diego Ferruzzo 
% Data: 28/10/2026
% =========================================================================
disp('Adicionar paths para as funçoes auxiliares');
addpath('../Runge-Kutta-Matlab/');
addpath('../Boeing-747-Matlab/');
disp('Feito!');
%
clear;
close all;
clc;
% 
disp('Simulaçao de um sistema de primeira ordem com controlador PID filtrado');
% Definiçao de um sistema de primeira ordem
function dy = primeira_ordem(y,u)
    a = 1;
    dy = a*y + u;
end
%
% parametros da simulaçao
t0 = 0;
tf = 10;
h = 1e-3;
y0 = 0.1;

n = floor((tf-t0)/h);
n_half = floor(n/2);
t = zeros(n,1);
y = zeros(n,1);
u = zeros(n,1);
e = zeros(n,1);
r = [zeros(n_half,1);ones(n_half,1)];

t(1) = t0;
y(1) = y0;

% Parametros do PID
Kp = 10;
Ki = 0.1; 
Kd = 0.1;
N = 1/h;
Ts = h;

e_hist = zeros(3,1);
u_hist = zeros(2,1);

for i=1:n-1
    %
    e(i) = r(i) - y(i);
    %
    e_hist = [e(i); e_hist(1:end-1)];
    u(i) = pid_filtrado(Kp, Ki, Kd, N, Ts, e_hist, u_hist);
    u_hist = [u(i); u_hist(1:end-1)];
    %
    f = @(t,y) primeira_ordem(y,u(i));
    [t(i+1), y(i+1)] = rk4(f, t(i), y(i), h);

end

disp('Feito!');

%% graficos

disp('Gerando graficos...');

figure(1);
plot(t,y,'-b');
hold on;
plot(t,e,'-r');
plot(t,u,'-g');
plot(t,r,'-k');
legend('y(t)','r(t)','u(t)','r(t)');
grid on;

disp('Feito!');
