close all 
clear all
%% Door simulation

% parameters
m = 1;
l1 = 0.3;
lc = 0.3;
d = 0.01;
Izz = m*((2*l1)^2+d^2)/12;
K = 4.16*l1;
B = 6.25*l1;

M = (m*l1^2 + Izz)/lc;


tspan = [0 15];
z0 = [0; 0];
Mu = 1;
F = 9.81/1.5;
% system
A = [ 0            1;
     -K/(M*l1)    -B/(M*l1)];

b = [ 0;
     1/M];
% 
ode = @(t,z) door_ode(t,z,F,A,b);
[t,z] = ode45(ode, tspan, z0);

% Plot solution
plot(t,z(:,1))
hold on
plot(t,z(:,2))
xlabel('t')
ylabel('solution y')

figure
plot(z(:,1),z(:,2))


function dzdt = door_ode(t,z,F,A,b)

 
dzdt = A*z + b*F;

end