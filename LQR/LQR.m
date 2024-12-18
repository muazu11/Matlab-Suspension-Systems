%%

clear;close all;clc
format long g; warning off;
set(0,'DefaultLineLineWidth',2);

%% Model Parameters

mb = 285; % Mass of vehicle (kg)
mw = 60;  % Mass of tire assembly (mw)
ks = 25400; % Rigidity of Suspension System (N/m)
bs = 1300; % Shock Absorber (N.s/m)
kt = 20e4; % Rigidity of tire (N/m)
fs = 12;  % Actuation force (N)

%% State Space Matrices

A = [0 1 0 0;
    [-ks -bs ks bs]/mb;
    0 0 0 1;
    [ks bs -ks-kt -bs]/mw];

B = [0 0;
     0 fs/mb;
     0 0;
     [kt -fs]/mw];

C = [1 0 0 0;
     1 0 -1 0;
     A(2,:)];

D = [0 0;
     0 0;
     B(2,:)];

sys = ss(A,B,C,D);

%% Controller

Q = [50000,0,0,0; 0,150000,0,0; 0,0,500000,0; 0,0,0,500000];

R = 5.15;

K_LQR = lqr(A,B(:,2),Q,R);

%% Test on nonlinear simulation

X0 = zeros(4,1);

zr = 5e-2; % Road disturbance (cm)

tsim = 4;

tt = 0:0.1:tsim;
kz = zeros(size(tt));
kz(tt>=0.5 & tt<=1.0) = 1;

Zr = [tt(:),kz(:)];

K = zeros(size(K_LQR));
open_loop = sim('QuarterCart_LQR_Control.slx',tsim);
K = K_LQR;
closed_loop = sim('QuarterCart_LQR_Control.slx',tsim);

figure(1);
set(gcf,'WindowState','Maximized');

subplot(2,2,1);hold on;grid on;box on
plot(tt,kz.*zr,'r.-');
plot(open_loop.tout,open_loop.x(:,1),'g.-');
plot(closed_loop.tout,closed_loop.x(:,1),'b--');
xlabel('t [sec]','FontSize',12,'FontWeight','Bold');
ylabel('$x_r\,[m]$','FontSize',12,'FontWeight','Bold','Interpreter','latex');
title('Car Body Travel','FontSize',12,'FontWeight','Bold');
legend('Road Bumb','Open Loop','Closed Loop','FontSize',12,'FontWeight','Bold');

subplot(2,2,2);hold on;grid on;box on
plot(open_loop.tout,open_loop.x(:,2),'g.-');
plot(closed_loop.tout,closed_loop.x(:,2),'b--');
xlabel('t [sec]','FontSize',12,'FontWeight','Bold');
ylabel('$x_b-x_w\,[m]$','FontSize',12,'FontWeight','Bold','Interpreter','latex');
title('Suspension Deflection','FontSize',12,'FontWeight','Bold');
legend('Open Loop','Closed Loop','FontSize',12,'FontWeight','Bold');

subplot(2,2,3);hold on;grid on;box on
plot(open_loop.tout,open_loop.x(:,4),'g.-');
plot(closed_loop.tout,closed_loop.x(:,4),'b--');
xlabel('t [sec]','FontSize',12,'FontWeight','Bold');
ylabel('$u\,[N]$','FontSize',12,'FontWeight','Bold','Interpreter','latex');
title('Control Force','FontSize',12,'FontWeight','Bold');
legend('Open Loop','Closed Loop','FontSize',12,'FontWeight','Bold');

subplot(2,2,4);hold on;grid on;box on
plot(open_loop.tout,open_loop.x(:,3),'g.-');
plot(closed_loop.tout,closed_loop.x(:,3),'b--');
xlabel('t [sec]','FontSize',12,'FontWeight','Bold');
ylabel('$\ddot{x_b}\,[m/s^2]$','FontSize',12,'FontWeight','Bold','Interpreter','latex');
title('Acceleration','FontSize',12,'FontWeight','Bold');
legend('Open Loop','Closed Loop','FontSize',12,'FontWeight','Bold');