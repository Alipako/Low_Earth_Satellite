function e = Quaternions2EulerAngles(quat)
%%%Introducem un vector de 4 obtine unul de 3

q0 = quat(1,:);
q1 = quat(2,:);
q2 = quat(3,:);
q3 = quat(4,:);
% Asta era din carte dar nu merge am gasit pe github
% phi = atan(2*(q0.*q1+q2*q3)/(1-2*(q1.^2+q2.^2)));
% theta = asin(2*(q0*q2-q3*q1));
% psi = atan(2*(q0*q3+q1+q2)/(1-2*(q2^2+q3^2)));


% roll (x-axis rotation)
sinr_cosp = 2.0 * (q0 .* q1 + q2 .* q3);
cosr_cosp = 1.0 - 2.0 * (q1 .* q1 + q2 .* q2);
phi = atan2(sinr_cosp, cosr_cosp);

% pitch (y-axis rotation)
sinp = 2.0 * (q0 .* q2 - q3 .* q1);
if (abs(sinp) >= 1)
  theta = pi/2*sign(sinp); % use 90 degrees if out of range
else
  theta = asin(sinp);
end

% yaw (z-axis rotation)
siny_cosp = 2.0 * (q0 .* q3 + q1 .* q2);
cosy_cosp = 1.0 - 2.0 * (q2 .* q2 + q3 .* q3);  
psi = atan2(siny_cosp, cosy_cosp);

e = [phi;theta;psi];