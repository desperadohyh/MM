%% test torque
clear all
close all

% robot parameters
dt          = 0.1;
% TB: obstacle
nobj        = 1;
obs         = {};
obs{1}.v    = [0;0];

xd = 0;
yd = 0;
obs{1}.poly = [1.1+xd 1.3+xd+0.5 1.4+xd+0.5 0.9+xd;0.1+yd 0.1+yd -0.5+yd -0.5+yd];
% TB: trajectory dimension
dim         = 2; %x,y
%%%%%%%%%%%%%%%%%%%%%%%%%%%
gen_ref_MMD
robot=robotproperty_MMD_s(4, z0_, Ts);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
m = robot.m;
lc = robot.lc;
l = robot.l;
Ic = robot.Ic;

A = [1 dt 0 0; 
     0  1 0 0;
     0  0 1 dt;
     0  0 0 1 ];
 
B = [0.5*dt^2    0 ;
     dt          0 ;
     0        0.5*dt^2;
     0           dt];




Z0 = sym('z',[4 1]);
robot.z0_ = [ 0 0 -pi/2 0 ]';

M_11 = m(1)*lc(1)^2+Ic{1}(3,3)+m(2)*l(1)^2+m(2)*lc(2)^2+Ic{2}(3,3)+2*m(2)*l(1)*lc(2)*cos(Z0(3));
M_12 = m(2)*lc(2)^2+Ic{2}(3,3)+m(2)*l(1)*lc(2)*cos(Z0(3));
M_21 = m(2)*lc(2)^2+Ic{2}(3,3)+m(2)*l(1)*lc(2)*cos(Z0(3));
M_22 = m(2)*lc(2)^2+Ic{2}(3,3);

M = [M_11 M_12;
     M_21 M_22];
 
V = [ -2*m(2)*l(1)*lc(2)*Z0(2)*sin(Z0(3))*Z0(4)-m(2)*l(1)*lc(2)*sin(Z0(3))*Z0(4)^2;
                         m(2)*l(1)*lc(2)*sin(Z0(3))*Z0(2)^2                       ];
 
G = [g*cos(Z0(1))*(m(1)*lc(1)+m(2)*l(1))+m(2)*g*lc(2)*cos(Z0(1)+Z0(3));
          m(2)*g*lc(2)*cos(Z0(1)+Z0(3))              ];
      


%% simulation loop

ss = 30;
t = 1:ss;
torque_implement = [];
torque_implement2 = [];
angle_implement = [];

robot.base = [0 0 0]';

% generat data       
% alpha_all = [-(pi/6)*cos((t/180)*pi*(180)/ss);
%              (pi/6)*cos((t/180)*pi*(180)/ss)];

%  alpha_all = [linspace(0.5236,0,ss/2) linspace(0,-0.5236,ss/2);
%               linspace(0,0.4,ss/3) linspace(0.4,-0.4,ss/3) linspace(-0.4,0,ss/3)];
% 

  alpha_all = [linspace(0,0.4,ss/3) linspace(0.4,-0.4,ss/3) linspace(-0.4,0,ss/3);
               linspace(0,0.4,ss/3) linspace(0.4,-0.4,ss/3) linspace(-0.4,0,ss/3)];
        

z0 = [pi/2 0 0 0]';

for steps = 1:ss

u = alpha_all(:,steps);
[zk]=lin_model(u,A,B,z0);

Mk = double(subs(M,Z0,zk));
Vk = double(subs(V,Z0,zk));
Gk = double(subs(G,Z0,zk));

torque_implement = [torque_implement  Mk*u+Vk+Gk ];
angle_implement = [angle_implement zk];
z0 = zk;
end

% % generat data
% z0_all = [ linspace(pi/2,pi/6,ss);
%            (pi/6)*sin((t/180)*pi*(180)/ss);
%            linspace(-pi/2,-pi/6,ss);
%            -(pi/6)*sin((t/180)*pi*(180)/ss)];
%        
% alpha_all = [(pi/6)*cos((t/180)*pi*(180)/ss);
%              -(pi/6)*cos((t/180)*pi*(180)/ss)];
%          
% for steps = 1:ss
% 
% robot.z0_ = z0_all(:,steps);
% 
% Mk = double(subs(M,Z0,robot.z0_));
% Vk = double(subs(V,Z0,robot.z0_));
% Gk = double(subs(G,Z0,robot.z0_));
% 
% torque_implement2 = [torque_implement2  Mk*alpha_all(:,steps)+Vk+Gk ];
% end

%% plot
gap = 1;
%figure
%plot_arm(ss,[angle_implement(1,:);angle_implement(3,:)],robot,gap,2);

%figure
plot(-torque_implement(1,:))
hold on
plot(-torque_implement(2,:))
hold on
xlabel('Time steps')
ylabel('Torque')
% plot(-torque_implement2(1,:))
% hold on
% plot(-torque_implement2(2,:))
legend('\tau \theta_1', '\tau \theta_2','o\theta_1', 'o\theta_2')

figure
angle_implement_deg = (180/pi).*angle_implement;
plot(angle_implement_deg(1,:))
hold on
%plot(angle_implement(2,:))
plot(angle_implement_deg(3,:))
%plot(angle_implement(4,:))
legend('\theta_1', '\theta_2')