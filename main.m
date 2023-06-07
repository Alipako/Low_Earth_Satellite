clc
clear
close all
tic

global invIs invI m I Is ms Ir1Bcg Ir2Bcg Ir3Bcg n1 n2 n3 maxSpeed maxAlpha Ir1B Ir2B Ir3B acc_rw euler_comandat pqr_comandat RPM_i sum_eror
%% Simularea unui satelit de tip CubeSat in orbita joasa a Pamantului
%%% Datele pentru Orbita si Satelit au fost preluate din cadrul satelitului
%%% ROSPIN-SAT-1


%% Date de intrare
Planet % Proprietatile planetei Pamant 
Inertia % Prorietatile inertiale ale satelitului  

%%% Conditiile initiale pozitie si viteze. ECEF (inertial) 
altitudine = 500000; % altitudinea orbitei in metri
x0 = R + altitudine; 
y0 = 0;
z0 = 0;
r_i0 = [x0;y0;z0]; % vectorul intial de poziti

x_der0 = 0; 
i_rad = deg2rad(97.49); % Inclinatia pentru orbita SSO
semi_major = norm([x0;y0;z0]); % orbtita fiind circulara 
vcircular = sqrt(mu/semi_major);
y_der0 = vcircular*cos(i_rad); 
z_der0 = vcircular*sin(i_rad);

v_i0 = [x_der0;y_der0;z_der0]; %vectorul de viteze initiale 

%%% Conditiile initiale pentru atitudine si viteze unghiulare 

phi0 = deg2rad(90); %rotit la grade in jurul axei x
theta0 = deg2rad(0); %rotit la grade in jurul axei y
psi0 = deg2rad(0); %rotit la grade in jurul axei z
euler0 = [phi0;theta0;psi0]; % La final acest euler -> sa-l redenumesc in euler
q0123_0 = euler2quat(euler0);
p0 = deg2rad(0); %initial roll rate rad/s
q0 = deg2rad(0); %initial pitch rate rad/s
r0 = deg2rad(0); %inital yaw rate rad/s


%%% Conditiile initiale RW 
w10 = 0;
w20 = 0;
w30 = 0;


%%% Toate conditiile initiale intr-un vecotr
state = [x0;y0;z0;x_der0;y_der0;z_der0;q0123_0;p0;q0;r0;w10;w20;w30]; 

%%% Perioada orbitala
C = cross(r_i0,v_i0);
p = norm(C)^2/mu;
L = -mu*r_i0/norm(r_i0) - cross(C,v_i0);
e = norm(L)/mu;
semi_major = p/(1-e^2);
r_max = p/(1-e);
r_min = p/(1+e);
period = 2*pi/sqrt(mu)*semi_major^(3/2);% 3 lege a lui Kepler (timpul in care se face o orbita) 


%%% Pregatim datele pentru durata si pasul simularii 
number_of_orbits = 1 % Numarul de orbite simulate 
delta_t = 0.1 % pasul de integrare [s]
tfinal = period*number_of_orbits
%tfinal = 100
tout = 0:delta_t:tfinal; 
stateout = zeros(length(tout),length(state));

%%% Data atitudinea dorita
euler_comandat = deg2rad([0;0;0]); % deg
pqr_comandat = deg2rad([0;0;0]); % deg/s
rwa = zeros(length(tout),3);
% %%% Consumul de curent initial 
ix = 0*stateout(:,1);
iy = 0*stateout(:,1);
iz = 0*stateout(:,1);


%%% Initializam variabilile
k1 = Dinamica(tout(1),state);

%% Integrarea ecuatiilor folosind RK4s
% [tout,stateout] = ode45(@Dinamica,tspan,stateinitial); Teste ode 45 
for index = 1:length(tout)
    %%% Salvam vectorii de stare 
    stateout(index,:) = state'; 
    
    %%% Consum RW
    RPM_rw = floor(abs(state(14:16,:)*60/2/pi));
    
    i_x(index) = RPM_i(RPM_rw(1)+1);
    i_y(index) = RPM_i(RPM_rw(3)+1);
    i_z(index) = RPM_i(RPM_rw(3)+1);

    %%%% Acceleratiile unghiulareale RW
    rwa(index,:) = acc_rw';
    
    %%% Cele 4 functii RK4
    k1 = Dinamica(tout(index),state);
    k2 = Dinamica(tout(index)+delta_t/2,state+k1*delta_t/2);
    k3 = Dinamica(tout(index)+delta_t/2,state+k2*delta_t/2);
    k4 = Dinamica(tout(index)+delta_t,state+k3*delta_t);
    k = (1/6)*(k1+2*k2+2*k3+k4);
    state = state + k*delta_t; 

end     

%% Calculam consumul total de curent
Consum_mWh = 8*(sum(i_x)/60 + sum(i_y)/60 +sum(i_z)/60)% mWh
%% Prelucrarea si afisarea graficilor 
stateout(:,1:6) = stateout(:,1:6)/1000; % m -> km 

%%% Extragem vectorul de stare
xout = stateout(:,1);
yout = stateout(:,2);
zout = stateout(:,3);
q0123_out = stateout(:,7:10)';

euler_out = quat2euler(q0123_out)';
euler_out_deg = rad2deg(euler_out);

pqr_out = stateout(:,11:13);
pqr_out_deg = rad2deg(pqr_out);
w123 = stateout(:,14:16)*60/2/pi; %RPM

%%% Generam o sfera 
[X,Y,Z] = sphere(50);
X = X*R/1000;
Y = Y*R/1000;
Z = Z*R/1000; 
load topo;

%%% Plotam orbita in 3D 
fig1 = figure();
set(fig1, 'color','white')
plot3(xout,yout,zout,'r-','LineWidth',4)
grid on
hold on 

rgb = imread('Corpuri_Ceresti\ear0xuu2.jpg');
rgb = im2double(flipdim(rgb,1));

surf(X,Y,Z, 'facecolor','texturemap',...
                   'cdata',rgb,...
                   'edgecolor','black');
axis equal

%%% Vizualizare unghiuri Euler 
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

plot(tout, euler_out_deg(:,1), 'parent', ax1,'LineWidth',2); hold on;
plot(tout, euler_out_deg(:,2), 'parent', ax2,'LineWidth',2); hold on;
plot(tout, euler_out_deg(:,3), 'parent', ax3,'LineWidth',2); hold on;

%%% Vizualizare viteze unghiulare 
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

plot(tout, pqr_out_deg(:,1), 'parent', ax4,'LineWidth',2); hold on;
plot(tout, pqr_out_deg(:,2), 'parent', ax5,'LineWidth',2); hold on;
plot(tout, pqr_out_deg(:,3), 'parent', ax6,'LineWidth',2); hold on;

%%% Vizualizare viteze RW
fig4 = figure;
ax7 = subplot(3,1,1);
axis ([-inf inf -inf inf]);
hold on 
grid on 
grid minor 
box on  
xlabel('Time [s]')
ylabel('X axis [RPM]')
title('RW speed');
ax8 = subplot(3,1,2);
axis ([-inf inf -inf inf]);
hold on 
grid on 
grid minor 
box on 
xlabel('Time [s]')
ylabel('Y axis [RPM]')
ax9 = subplot(3,1,3);
axis ([-inf inf -inf inf]);
hold on 
grid on 
grid minor 
box on   
xlabel('Time [s]')
ylabel('Z axis [RPM]')

plot(tout, w123(:,1), 'parent', ax7,'LineWidth',2); hold on;
plot(tout, w123(:,2), 'parent', ax8,'LineWidth',2); hold on;
plot(tout, w123(:,3), 'parent', ax9,'LineWidth',2); hold on;

%%% Vizualizare consum RW
fig5 = figure;
ax10 = subplot(3,1,1);
axis ([-inf inf -inf inf]);
hold on 
grid on 
grid minor 
box on  
xlabel('Time [s]')
ylabel('X axis [mA]')
title('RW current');
ax11 = subplot(3,1,2);
axis ([-inf inf -inf inf]);
hold on 
grid on 
grid minor 
box on 
xlabel('Time [s]')
ylabel('Y axis [RPM]')
ax12 = subplot(3,1,3);
axis ([-inf inf -inf inf]);
hold on 
grid on 
grid minor 
box on   
xlabel('Time [s]')
ylabel('Z axis [RPM]')

plot(tout, i_x, 'parent', ax10,'LineWidth',2); hold on;
plot(tout, i_y, 'parent', ax11,'LineWidth',2); hold on;
plot(tout, i_z, 'parent', ax12,'LineWidth',2); hold on;



% %The End
% load gong.mat;
% gong = audioplayer(y, Fs);
% play(gong);

toc
