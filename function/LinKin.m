function [Z1, A, B ] = LinKin(z0_, U0, Ts)
%z0_ = zeros(1,19);
%Ts = 1;
r = 0.03;
d = 0.15;
Z6_ = z0_(6)+.5*Ts*z0_(7);
Interg = [1 Ts;
          0  1];
      
Interg_ = [0.5*Ts^2;
              Ts];
A = [ 1   0    0    0  Ts*cos(Z6_) -z0_(5)*Ts*sin(Z6_) -0.5*(Ts^2)*z0_(5)*sin(Z6_) zeros(1,12);
      0   1    0    0  Ts*sin(Z6_)  z0_(5)*Ts*cos(Z6_)  0.5*(Ts^2)*z0_(5)*cos(Z6_) zeros(1,12);
      zeros(1,5)        -0.5*r*(z0_(9)+z0_(11))*sin(z0_(7)) 0 0 r*0.5*cos(z0_(7)) 0 r*0.5*cos(z0_(7)) zeros(1,8);
      zeros(1,5)        0.5*r*(z0_(9)+z0_(11))*cos(z0_(7)) 0 0 r*0.5*sin(z0_(7)) 0 r*0.5*sin(z0_(7)) zeros(1,8);
      zeros(1,8)           0.5*r  0 0.5*r zeros(1,8);
      zeros(1,5)             1 Ts zeros(1,12);
      zeros(1,8)           r/d 0 -r/d zeros(1,8);
      zeros(12,7)           kron(eye(6),Interg)];
  
B = [ zeros(7,6);
     kron(eye(6),Interg_)];
 
Z1 = A*z0_ + B*U0;