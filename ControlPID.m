function [acc_rw] = ControlPID(pqr,euler,euler_comandat,pqr_comandat)

% Importam datele rotilor inertiale 
reaction_wheel_params

% Setam Kp si Kd  
% v1 sistem rapid si precis
KP = eye(3)*0.0803; 
KD = eye(3)*0.1376; 
% v2 sistem eficient 
% KP = eye(3)* 0.00011466;
% KD = eye(3)* 0.003210;


% Calculam Cuplul necesar 
% Mdesired = -KD*(pqr_comandat - pqr) - KP*(euler_comandat-euler);
Tdesired = KP*(euler - euler_comandat) + KD*(pqr - pqr_comandat) ; % ec. (138) 

% Extragem acceleratia unghiulara a pentru fiecare roata 
acc_rw = Jinv*Tdesired; % https://en.wikipedia.org/wiki/Torque