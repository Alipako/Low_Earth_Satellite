%% Calculul momentelor de inertie pentru un CubeSat 3U. 
%%% [body frame] cu originea in centrul de masa 
global invIs invI m I Is ms Ir1Bcg Ir2Bcg Ir3Bcg n1 n2 n3 maxSpeed maxAlpha Ir1B Ir2B Ir3B acc_rw euler_comandat pqr_comandat

%%% Consideram ca masa este distribuita uniform in satelit
ms = 3; % Kg
a = 0.1; % x
b = 0.1; % y
c = 0.3; % z
Is = [m*(b^2+c^2)/12 0 0; 0 m*(a^2+c^2)/12 0; 0 0 m*(a^2+b^2)/12];
invIs=inv(I);

%%% Intorucem parametrii RW
reaction_wheel_params

%%% Inertia totala 
m = ms + 3*mr;
I = Is + Ir1Bcg + Ir2Bcg + Ir3Bcg;

%%% Matricea inversa
invI = inv(I);

