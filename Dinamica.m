function derivate = Dinamica(t,state)
%stateinitial = [x0;y0;z0;x_der0;y_der0;z_der0;q0123;p0;q0;r0];
global invIs invI m I Is ms Ir1Bcg Ir2Bcg Ir3Bcg n1 n2 n3 maxSpeed maxAlpha Ir1B Ir2B Ir3B acc_rw euler_comandat pqr_comandat RPM_i sum_eror

%x = state(1);
%y = state(2);
%z = state(3);
%xdot = state(4);
%ydot = state(5);
%zdot = state(6);
q0123 = state(7:10);
euler = quat2euler(q0123);
p = state(11);
q = state(12);
r = state(13);
pqr = state(11:13);
w123 = state(14:16);

%%% Cinematica - Translatie
vel = state(4:6);

%%% Cinematica - Rotatie
quadmat = [0 -p -q -r; 
           p 0 r -q; 
           q -r 0 p; 
           r q  -p 0];
q0123_der = 0.5*quadmat*q0123;

%%% Modelul Gravitational pentru corpul de referinta
Planet 
r = state(1:3); % r = [x;y;z]
rnorm = norm(r); % distanta vectorului
rhat=r/rnorm ; % directia vectorului
Fgrav= -(G*M*m/rnorm^2)*rhat;

%%% Controlul satelitului 
[acc_rw] = ControlPID(pqr,euler,euler_comandat,pqr_comandat);

%%% Modelul matematic al Rotilor Inertiale 
% Am impus 2 conditii:
% Viteza unghiulara maxima 
% Acceleratia unghiulara maxima  

w123dot = [0;0;0];
for index = 1:3
    if abs(w123(index)) > maxSpeed 
        w123dot(index) = 0; 
    else
        if abs(acc_rw(index)) > maxAlpha 
            acc_rw(index) = sign(acc_rw(index))*maxAlpha;
        end
        w123dot(index) = acc_rw(index);
    end
end
mom_RWs = Ir1B*w123dot(1)*n1 + Ir2B*w123dot(2)*n2 + Ir3B*w123dot(3)*n3;


%%% Actutarori (pana cand am doar RW) 
Total_mom = - mom_RWs;

%%% Dinamica - Translatie 
F = Fgrav;
accel = F/m;

%%% Momentul unghiular total 
w1 = w123(1); % viteza unghiulara roata 1
w2 = w123(2); % viteza unghiulara roata 2
w3 = w123(3); % viteza unghiulara roata 3
H = Is*pqr + Ir1B*w1*n1 + Ir2B*w2*n2 + Ir3B*w3*n3; % ec. 60

%%% Dynamica - Rotatie
pqr_der = invI*(Total_mom - cross(pqr,H));

%%% Vectorul starilor derivate
derivate = [vel;accel;q0123_der;pqr_der;w123dot];
