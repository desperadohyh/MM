% dt = 0.1;
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%
% gen_ref_MMD
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% % Arm definition
% % Arm parameters
% robot=robotproperty_MMD(4, z0_, Ts);
% nlink = 5;
% 
% 
% robot.Z0 = sym('z',[19 1]);
% robot.z0_ = sym('zk',[19 1]);



% robot.Ic = cell(1,nlink);
% 
% for i = 2:nlink
%     robot.Ic{i} = diag(robot.Ixyz(:,i-1));
% end
% robot.Ic{1} = sym('IC1_',[3,3]);
%%

% [ Mk, Vk, Gk, robot] = get_joint_torque_sym(robot)
% Mkf =  matlabFunction(Mk,'File','Mk_f');
% Vkf =  matlabFunction(Vk,'File','Vk_f');
% Gkf =  matlabFunction(Gk,'File','Gk_f');
%%
%load('MVG_2.mat')
ss = 10;
t = 1:ss;
torque_implement = [];
torque_implement1 = [];
angle_implement = [];
robot.base = [0 0 0]';

z0_k = [0 0 0 0 0    0 0 0 0 0     0 -pi/2 0 0 0      0 0 0 0 ]';
% generate refrence

alpha_all = [ 8*(pi)*cos((t/180)*pi*(180)/ss);
              (pi)*cos((t/180)*pi*(180)/ss);
              3*(pi)*cos((t/180)*pi*(180)/ss);
              (pi)*cos((t/180)*pi*(180)/ss);
             -(pi/2)*cos((t/180)*pi*(180)/ss);
             -(pi/3)*cos((t/180)*pi*(180)/ss)  ];  
         
for steps = 1:ss
    
  steps  
u = alpha_all(:,steps);

[zk, A, B ] = LinKin(z0_k, u, dt);


Mk_Z = Mk_f(zk(6),zk(12),zk(14),zk(16),zk(18),zk(6),zk(9),zk(11));
Vk_Z = Vk_f(zk(6),zk(7),zk(9),zk(11),zk(12),zk(13),zk(14),zk(15),zk(16),zk(17),zk(18),zk(19),zk(6),zk(9),zk(11),zk(13),zk(15),zk(17),zk(19));
Gk_Z = Gk_f(zk(14),zk(16),zk(18));



tau = Mk_Z*u+Vk_Z+Gk_Z;


torque_implement = [torque_implement  double(tau) ];

% [ A_tau, b_tau, robot] = joint_torque_old(robot,obs_arm);
% torque_implement1 = [torque_implement1  A_tau*alpha_all(:,steps)+b_tau ];
angle_implement = [angle_implement zk];
z0_k = zk;
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
gap = 2;
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

figure
angle_implement_deg = (180/pi).*angle_implement;
plot(angle_implement_deg(8,:))
hold on
plot(angle_implement_deg(10,:))
plot(angle_implement_deg(12,:))
plot(angle_implement_deg(14,:))
plot(angle_implement_deg(16,:))
plot(angle_implement_deg(18,:))

legend('\theta_1', '\theta_2','\theta_3', '\theta_4','\theta_5', '\theta_6')
