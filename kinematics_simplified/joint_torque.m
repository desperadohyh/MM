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
robot=robotproperty_MMD(4, z0_, Ts);
%%
% Arm joint
njoint      =5; % joint number
nstate      =10; % QP variable dim
nu          =5; % acceleration dim 
DH          =robot.DH;


bb=1;
robot.Z0 = sym('z',[19 1]);
robot.z0_ = zeros(19,1);
robot.nTherta = 6;

%% simulation loop
ss = 1;
t = 1:ss;
torque_implement = [];
torque_implement1 = [];
angle_implement = [];
robot.base = [0 0 0]';


% generate refrence

alpha_all = [ 0*(pi)*cos((t/180)*pi*(180)/ss);
              0*(pi)*cos((t/180)*pi*(180)/ss);
              3*(pi)*cos((t/180)*pi*(180)/ss);
              (pi/6)*cos((t/180)*pi*(180)/ss);
             -(pi/2)*cos((t/180)*pi*(180)/ss);
             -(pi/3)*cos((t/180)*pi*(180)/ss)  ];   



for steps = 1:ss
    
  steps  
u = alpha_all(:,steps);

[zk, A, B ] = LinKin(z0_, u, dt);
robot.z0_ = zk;
[ Mk, Vk, Gk, robot] = get_joint_torque_new(robot);



torque_implement = [torque_implement  Mk*u+Vk+Gk ];
% [ A_tau, b_tau, robot] = joint_torque_old(robot,obs_arm);
% torque_implement1 = [torque_implement1  A_tau*alpha_all(:,steps)+b_tau ];
angle_implement = [angle_implement zk];
z0_ = zk;
end

%% Plot
traj_implement = [angle_implement(1,:);
                  angle_implement(2,:)];

theta_implement = [angle_implement(6,:);
                   angle_implement(12,:);
                   angle_implement(14,:);
                   angle_implement(16,:);
                   angle_implement(18,:);
                   0.07*ones(1,ss)      ];
figure
gap = 5;
plot_MM5(ss,theta_implement(1:end-1,:),traj_implement,robot,tb3,gap,0)
%plot_implement(ss,theta_implement,traj_implement,robot,tb3,gap,0)

figure
plot(torque_implement(1,:))
hold on
plot(torque_implement(2,:))
plot(torque_implement(3,:))
plot(torque_implement(4,:))
plot(torque_implement(5,:))
plot(torque_implement(6,:))

xlabel('Time steps')
ylabel('Torque')
% plot(torque_implement1(1,:))
% hold on
% plot(torque_implement1(2,:))
legend('\theta_1', '\theta_2','\theta_3', '\theta_4','\theta_5', '\theta_6')


