clear 
clc


global invIs invI m I Is ms Ir1Bcg Ir2Bcg Ir3Bcg n1 n2 n3 maxSpeed maxAlpha Ir1B Ir2B Ir3B acc_rw euler_comandat pqr_comandat RPM_i
%function [acc_rw] = ControlLQR(pqr,euler,euler_comandat,pqr_comandat)

% Initial Cond

x0 = [90 0 0 0 0 0]; % [phi theta psi p q r]

% Importam datele rotilor inertiale 
reaction_wheel_params
Inertia 
w_0 = 0.0010764;
% Definim linear State-Space equation - ecuatia linearizata in jurul w_0
A = [0 0 0 1 0 0;
     0 0 0 0 1 0; 
     0 0 0 0 0 1; 
     4*w_0^2*(I(3,3)-I(2,2))/I(1,1) 0 0 0 0 w_0*(I(3,3)-I(2,2)+I(3,3))/I(1,1);
     0 3*w_0^2*(I(1,1)-I(3,3))/I(2,2) 0 0 0 0;
     0 0 w_0^2*(I(1,1)-I(2,2))/I(3,3) w_0*(I(2,2)-I(1,1)-I(3,3))/I(3,3) 0 0];

B = [0 0 0;
     0 0 0; 
     0 0 0;
     1/I(1,1) 0 0;
     0 1/I(2,2) 0;
     0 0 1/I(3,3)];

C = [0 0 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1];

D = 0 ;

% Cost Function Design
%v1 Prioritizam executia si precizia 
% Q = diag([100, 100, 100, 100, 100, 100]); % Penalizam eroarea phi theta psi p q r
% R = diag([1, 1, 1]); % Penalizam momentul unghiular al rotilor 

%v2 Prioritizam consumul de energie 
Q = diag([1, 1, 1, 1, 1, 1]); % Penalizam eroarea phi theta psi p q r
R = diag([100, 100, 100]); % Penalizam momentul unghiular al rotilor 

% LQR Controller Design
K = lqr(A,B,Q,R)

% Closed loop system 
sys = ss((A - B*K), B, C, D);

% Run
t = 0:0.01:10; % Time span for simulation
[y,t,x] = initial(sys, x0, t);



% Plotting Results
fig2 = figure;
ax1 = subplot(3,1,1);
axis ([-inf inf -inf inf]);
hold on 
grid on 
grid minor 
box on  
xlabel('Time [s]')
ylabel('X axis [deg]')
title('Euler Angles');
ax2 = subplot(3,1,2);
axis ([-inf inf -inf inf]);
hold on 
grid on 
grid minor 
box on 
xlabel('Time [s]')
ylabel('Y axis [deg]')
ax3 = subplot(3,1,3);
axis ([-inf inf -inf inf]);
hold on 
grid on 
grid minor 
box on   
xlabel('Time [s]')
ylabel('Z axis [deg]')

plot(t, x(:,1), 'parent', ax1,'LineWidth',2); hold on;
plot(t, x(:,2), 'parent', ax2,'LineWidth',2); hold on;
plot(t, x(:,3), 'parent', ax3,'LineWidth',2); hold on;

% Viteze unghiulare
fig3 = figure;
ax4 = subplot(3,1,1);
axis ([-inf inf -inf inf]);
hold on 
grid on 
grid minor 
box on  
xlabel('Time [s]')
ylabel('X axis [deg/s]')
title('Angular speed');
ax5 = subplot(3,1,2);
axis ([-inf inf -inf inf]);
hold on 
grid on 
grid minor 
box on 
xlabel('Time [s]')
ylabel('Y axis [deg/s]')
ax6 = subplot(3,1,3);
axis ([-inf inf -inf inf]);
hold on 
grid on 
grid minor 
box on   
xlabel('Time [s]')
ylabel('Z axis [deg/s]')

plot(t, x(:,4), 'parent', ax4,'LineWidth',2); hold on;
plot(t, x(:,5), 'parent', ax5,'LineWidth',2); hold on;
plot(t, x(:,6), 'parent', ax6,'LineWidth',2); hold on;

% Cuplu 
fig4 = figure;
ax7 = subplot(3,1,1);
axis ([-inf inf -inf inf]);
hold on 
grid on 
grid minor 
box on  
xlabel('Time [s]')
ylabel('X axis [Nm]')
title('Torque');
ax8 = subplot(3,1,2);
axis ([-inf inf -inf inf]);
hold on 
grid on 
grid minor 
box on 
xlabel('Time [s]')
ylabel('Y axis [Nm]')
ax9 = subplot(3,1,3);
axis ([-inf inf -inf inf]);
hold on 
grid on 
grid minor 
box on   
xlabel('Time [s]')
ylabel('Z axis [Nm]')

plot(t, y(:,4), 'parent', ax7,'LineWidth',2); hold on;
plot(t, y(:,5), 'parent', ax8,'LineWidth',2); hold on;
plot(t, y(:,6), 'parent', ax9,'LineWidth',2); hold on;
