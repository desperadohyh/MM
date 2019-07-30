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
dt          = 0.5;
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
robot=robotproperty_MMD_s(4, z0_, Ts);
% Arm joint
njoint      =5; % joint number
nstate      =10; % QP variable dim
nu          =5; % acceleration dim 
DH          =robot.DH;

% Arm obs
% center position (1.05,-0.2)
obs_arm     =[[1.05;-0.2;0.35] [1.05;-0.2;0.5]];
obs_arm_r   = 0.35; % radius

ss=5;
X_out(1) = 0;
cc = 1;
xV = 0.2

bb=1;
robot.Z0 = sym('z',[4 1]);
robot.z0_ = [ 0 0 -pi/2 0 ]';
robot.nTherta = 6;

%% simulation loop
ss = 30;
t = 1:ss;
torque_implement = [];
torque_implement1 = [];
robot.base = [0 0 0]';
% generate refrence
z0_all = [ linspace(0,pi/3,ss);
           (pi/6)*sin((t/180)*pi*(180)/ss);
           linspace(0,-pi/3,ss);
           -(pi/6)*sin((t/180)*pi*(180)/ss)];
       
alpha_all = [(pi/6)*cos((t/180)*pi*(180)/ss);
             -(pi/6)*cos((t/180)*pi*(180)/ss)];
           
    



for steps = 1:ss

robot.z0_ = z0_all(:,steps);
[ Mk, Vk, Gk, robot] = joint_torque_r(robot,obs_arm);
torque_implement = [torque_implement  Mk*alpha_all(:,steps)+Vk+Gk ];
[ A_tau, b_tau, robot] = joint_torque_old(robot,obs_arm);
torque_implement1 = [torque_implement1  A_tau*alpha_all(:,steps)+b_tau ];
end
%% plot
figure
gap = 3;
plot_arm(ss,[z0_all(1,:);z0_all(3,:)],robot,gap,1)

figure
plot(torque_implement(1,:))
hold on
plot(torque_implement(2,:))

xlabel('Time steps')
ylabel('Torque')
plot(torque_implement1(1,:))
hold on
plot(torque_implement1(2,:))
legend('\theta_1', '\theta_2','o\theta_1', 'o\theta_2')