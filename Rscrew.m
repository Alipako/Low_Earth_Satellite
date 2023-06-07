function R = Rscrew(nhat)

x = nhat(1);
y = nhat(2);
z = nhat(3);

%%%yaw 
ps = atan2(y,x);
%%%Theta
theta = atan2(z,sqrt(x^2+y^2));
%%%Phi is always zero because it is a line
phi = 0;

R = TIB(phi,theta,ps);


% Copyright - Carlos Montalvo 2015
% You may freely distribute this file but please keep my name in here
% as the original owner