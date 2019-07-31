clc
clear all
close all
% fighandle = [];
% fighandle(1) = figure(1); hold on;
% set(gcf, 'position', [0 0 500 500]);
% fighandle(2) = figure(2); hold on;

    
%% parameter definition
% TB definition
% sampling time
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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Arm definition
% Arm parameters
robot=robotproperty_MMD_4(4, z0_, Ts);
% Arm joint
njoint      =5; % joint number
nstate      =10; % QP variable dim
nu          =5; % acceleration dim 
DH          =robot.DH;



bb=1;
robot.Z0 = sym('z',[4 1]);
robot.z0_ = [ 0 0 -pi/2 0 ]';
robot.nTherta = 6;

%% simulation loop
ss = 30;
t = 1:ss;
torque_implement = [];
torque_implement1 = [];
angle_implement = [];
robot.base = [0 0 0]';

Interg = [1 dt;
          0  1];
      
Interg_ = [0.5*dt^2;
              dt];

A = kron(eye(4),Interg);
 
B = kron(eye(4),Interg_);
% generate refrence

alpha_all = [            zeros(1,ss);
              0*(pi/6)*cos((t/180)*pi*(180)/ss);
             -0*(pi/6)*cos((t/180)*pi*(180)/ss);
                         zeros(1,ss)           ];
           
    

z0 = [0 0 0 0 -pi/2 0 0 0]';

for steps = 1:ss
    
    
u = alpha_all(:,steps);
[zk]=lin_model(u,A,B,z0);

robot.z0_ = zk;
[ Mk, Vk, Gk, robot] = joint_torque_r(robot,obs_arm);
torque_implement = [torque_implement  Mk*alpha_all(:,steps)+Vk+Gk ];
% [ A_tau, b_tau, robot] = joint_torque_old(robot,obs_arm);
% torque_implement1 = [torque_implement1  A_tau*alpha_all(:,steps)+b_tau ];
angle_implement = [angle_implement zk];
z0 = zk;
end
%% plot
figure
gap = 3;
plot_arm(ss,[angle_implement(1,:);angle_implement(3,:)],robot,gap,1)

figure
plot(torque_implement(1,:))
hold on
plot(torque_implement(2,:))

xlabel('Time steps')
ylabel('Torque')
% plot(torque_implement1(1,:))
% hold on
% plot(torque_implement1(2,:))
legend('\theta_1', '\theta_2')