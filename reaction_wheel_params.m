%% Aici definim toti parametrii rotilor inertiale preluate din : https://www.cubespace.co.za/products/gen-1/actuators/cubewheel/
% Medium CubeWheel
% Parametrii rotilor inertiale 
mr = 0.13; %[kg]
rr = 42/1000; % raza rotii [m]
hr = 19/1000; % inaltimea rotii [m] 
 
% Viteza maxima de rotatie
rpm = 6000; % [RPM]
maxSpeed = rpm*2*pi/60; % [rad/s]

%%% Cuplu maxim 
maxTorque = 0.004; % [Nm]

% Orientarea rotilor - o roata in fiecare directie 
n1 = [1;0;0];
n2 = [0;1;0];
n3 = [0;0;1];

% Distanta intre CG si rotile ineritale 
r1 = [4;0;0]/1000;
r2 = [0;4;0]/1000;
r3 = [0;0;4]/1000;

% Calculam momentele de inertie 
Idisk = (1/12)*(3*rr^2+hr^2);
IrR = mr*[(1/2)*rr^2 0 0 ;0 Idisk 0; 0 0 Idisk];

%%% Calculam acceleratia unghiulara maxima
maxAlpha = maxTorque/IrR(1,1);

%%%  Transformam din RW in body frame
T1 = Rscrew(n1);
T2 = Rscrew(n2);
T3 = Rscrew(n3);

%%% Calculam inertia RW in sistemul de referinta a satelitului (body frame)
Ir1B = T1'*IrR*T1;
Ir2B = T2'*IrR*T2;
Ir3B = T3'*IrR*T3;

%%% Aici matricea J de inertie ne trebuie pentru a calcula acceleratia unghiulara
J = [Ir1B*n1,Ir2B*n2,Ir3B*n3];
Jinv = inv(J'*J)*J'; % ec. 140 

%%% Calculez inertia RW in sistemul de referinta body frame in referinta
%%% fata de centrul de greutate a satelitului folosind Stainer (Parallel axis theorem)
sr1 = skew(r1);
Ir1Bcg = Ir1B + mr*(sr1')*sr1;
sr2 = skew(r2);
Ir2Bcg = Ir2B + mr*(sr2')*sr2;
sr3 = skew(r3);
Ir3Bcg = Ir3B + mr*(sr3')*sr3;

%%% Consumul RW 
% tensiune 8V
datasheet_viteze = [0 2000 6000];
datasheet_curent = [0 16.25 287.5];
rot_tot = 0:1:6000;

RPM_i = interp1(datasheet_viteze, datasheet_curent, rot_tot, "pchip");% RPM_i(RPM)=i [mA]
% figure
% plot(datasheet_viteze,datasheet_curent,'o',rot_tot,RPM_i);
% legend('datasheet','consum');
% xlabel('RW speed [RPM]')
% ylabel('Current [I]')
% title('RW current consumption');
% grid on
% grid minor 
% box on  
