




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

% initialize
t_now = 0;
z_now = [0 ;0];
z_stop = false;
sample = 2;

t_all = [];
z_all = [];
aa = 0;


% system
A = [ 0            1;
     -K/(M*l1)    -B/(M*l1)];

b = [ 0;
     1/M];
 


% adaptive loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while t_now<6

    % update
    tspan = [t_now 15+t_now];
    z0 = [z_now(1); z_now(2)];
    
    
ode = @(t,z) door_ode(t,z,F,A,b);
[t,z] = ode45(ode, tspan, z0);

if z(sample,1)>1.18 && z_stop == false
    z_stop_th = z(sample,1);
    t_stop = t(sample);
    z_stop = true;
    F = 9.81/2;
    sample = 20;
    t(sample)=t_stop+0.01
end


if z_stop == true  
    
    z = [ones(size(z,1),1)*z_stop_th zeros(size(z,1),1)];  
    z_now = z(sample,:)'
    pause
end

t_now = t(sample);
z_now = z(sample,:)'

t_all = [t_all; t_now];
z_all = [z_all; z_now'];
aa=aa+1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
% Plot solution
figure
plot(t_all,z_all(:,1))
hold on
plot(t_all,z_all(:,2))
% plot(t,z(:,1))
% hold on
% plot(t,z(:,2))
xlabel('t')
ylabel('solution y')

% figure
% plot(z(:,1),z(:,2))


function dzdt = door_ode(t,z,F,A,b)

 
dzdt = A*z + b*F;

end