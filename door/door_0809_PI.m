
clc
clear all
close all
fighandle = [];
fighandle(1) = figure(1); hold on;
set(gcf, 'position', [0 0 500 500]);
fighandle(2) = figure(2); hold on;

%% Door simulation

% parameters

m = 1;
l1 = 0.3;
lc = 0.3;
d = 0.01;
Izz = m*((2*l1)^2+d^2)/12;
% K = 4.16*l1;
% B = 6.25*l1;
K = 2.16*l1;
B = 2.25*l1;

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
X_all = [];
aa = 0;


% system
A = [ 0            1;
     -K/(M*l1)    -B/(M*l1)];

b = [ 0;
     1/M];

 F_all = [];


% adaptive loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while t_now<6

    % update
    tspan = [t_now 15+t_now];
    tt = linspace(t_now,t_now+2,21);
    z0 = [z_now(1); z_now(2)];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% transfer function

% Fc = tf([1/M],[1,B/M,K/M],0.05);
% dthc = tf([1],[1,B/M,K/M],0.05);
% thc = tf([1,B/M],[1,B/M,K/M],0.05);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
ode = @(t,z) door_ode(t,z,F*lc,A,b);
[t,z] = ode45(ode, tt, z0);

% if z(sample,1)>0.9 && z_stop == false
%     z_stop_th = z(sample,1);
%     t_stop = t(sample);
%     z_stop = true;
%     F = 9.81/2;
%     sample = 20;
%     t(sample)=t_stop+0.01
%     pause
% end


if z_stop == true  
    
    z = [ones(size(z,1),1)*z_stop_th zeros(size(z,1),1)];  
    z_now = z(sample,:)';
    %pause
end

t_now = t(sample);
z_now = z(sample,:)';

% fighandle(1)
% plot(t,z(:,1)')
% hold on
% pause

t_all = [t_all; t_now];
z_all = [z_all; z_now'];
aa=aa+1;


lf = Rgl_f(z_now(1))*pinv(Jk_f(z_now(1)))'*F*lc;
F_all = [ F_all lf];

% generate xdd xd x reference

xdd = dJk_f(z_now(1),z_now(2))*z_now(2) + Jk_f(z_now(1))*(F*lc/M);
xd = Jk_f(z_now(1))*z_now(2);
x = [lc*cos(z_now(1));
     lc*sin(z_now(1));
         0          ];
X_all = [X_all [x; xd; xdd]];

end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
% Plot solution
fighandle(1)
plot(t_all,z_all(:,1))
hold on
plot(t_all,z_all(:,2))
% plot(t,z(:,1))
% hold on
% plot(t,z(:,2))
xlabel('t')
ylabel('solution y')

% plot trajectory
figure
plot3(X_all(1,:),X_all(2,:),X_all(3,:))
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
axis equal
axis([-0.5 0.7 -0.3 0.5 0 0.4])

% plot velosity
figure
plot(t_all,X_all(4:6,:)')
xlabel('Time [s]')
ylabel('Velosity [m/s]')
legend('V_x','V_y','V_z')

% plot acceleration
figure
plot(t_all,X_all(7:9,:)')
xlabel('Time [s]')
ylabel('Acceleration [m/s^2]')
legend('a_x','a_y','a_z')


% figure
figure
plot(F_all')
legend('F_{x,l}','F_{y,l}','F_{z,l}')
% plot(z(:,1),z(:,2))


function dzdt = door_ode(t,z,F,A,b)

 
dzdt = A*z + b*F;

end