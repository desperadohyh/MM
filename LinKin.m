function [Z1, A, B ]= LinKin(Z0, U0, Ts)
%Z0 = zeros(1,21);
%Ts = 1;
r = 0.03;
d = 0.15;
Z6_ = Z0(6)+.5*Ts*Z0(7);
Interg = [1 Ts;
          0  1];
      
Interg_ = [0.5*Ts^2;
              Ts];
A = [ 1   0    0    0  Ts*cos(Z6_) -Z0(5)*Ts*sin(Z6_) -0.5*(Ts^2)*Z0(5)*sin(Z6_) zeros(1,12);
      0   1    0    0  Ts*sin(Z6_)  Z0(5)*Ts*cos(Z6_)  0.5*(Ts^2)*Z0(5)*cos(Z6_) zeros(1,12);
      zeros(1,5)        -0.5*r*(Z0(9)+Z0(11))*sin(Z0(7)) 0 0 r*0.5*cos(Z0(7)) 0 r*0.5*cos(Z0(7)) zeros(1,8);
      zeros(1,5)        0.5*r*(Z0(9)+Z0(11))*cos(Z0(7)) 0 0 r*0.5*sin(Z0(7)) 0 r*0.5*sin(Z0(7)) zeros(1,8);
      zeros(1,8)           0.5*r  0 0.5*r zeros(1,8);
      zeros(1,5)             1 Ts zeros(1,12);
      zeros(1,8)           r/d 0 -r/d zeros(1,8);
      zeros(12,7)           kron(eye(6),Interg)];
  
B = [ zeros(7,6);
     kron(eye(6),Interg_)];
 
Z1 = A*Z1 + B*U0;
