clc
clear all
close all

dt = 0.1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%
gen_ref_MMD_0726
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Arm parameters
robot=robotproperty_MMD(4, z0_, Ts);
nlink = 5;


%%
%load('MVG_3.mat')
ss = 10;
t = 1:ss;
torque_implement = [];
torque_implement1 = [];
angle_implement = [];
forcel_implement = [];
forceg_implement = [];
robot.base = [0 0 0]';

z0_k = [0 0 0 0 0    0 0 0 0 0     0 -pi/2 0 0 0      0 0 0 0 ]';
% generate refrence

inputi = 3;
switch inputi
    case 1

alpha_all = [ 8*(pi)*cos((t/180)*pi*(180)/ss);
              (pi)*cos((t/180)*pi*(180)/ss);
              3*(pi)*cos((t/180)*pi*(180)/ss);
              (pi)*cos((t/180)*pi*(180)/ss);
             -(pi/2)*cos((t/180)*pi*(180)/ss);
             -(pi/3)*cos((t/180)*pi*(180)/ss)  ]; 

    case 2
         alpha_all = [ 0*(pi)*cos((t/180)*pi*(180)/ss);
              0*(pi)*cos((t/180)*pi*(180)/ss);
                0*3*(pi)*cos((t/180)*pi*(180)/ss);
              (pi)*cos((t/180)*pi*(180)/ss);
             -(pi)*cos((t/180)*pi*(180)/ss);
             -0*(pi/3)*cos((t/180)*pi*(180)/ss)  ];   
    case 3
         alpha_all = [ 0*(pi)*cos((t/180)*pi*(180)/ss);
              0*(pi)*cos((t/180)*pi*(180)/ss);
              0*3*(pi)*cos((t/180)*pi*(180)/ss);
              0*(pi)*cos((t/180)*pi*(180)/ss);
             -0*(pi/2)*cos((t/180)*pi*(180)/ss);
             -0*(pi/3)*cos((t/180)*pi*(180)/ss)  ]; 
end
         
% initials
load('open_data.mat')
pend = [-0.1500 -0.2400 0.2540]';
dpend = [0  0  0]';
zk_d0 = z0_k;


         
for steps = 1:ss

% desired reference    
% u_d = alpha_all(:,steps);
% [zk_d, A, B ] = LinKin(zk_d0, u, dt);

ddx_d
dx_d
x_d


Mk_Z = Mk_f(zk(6),zk(12),zk(14),zk(16),zk(18));
Vk_Z = Vk_f(zk(3),zk(4),zk(6),zk(7),zk(12),zk(13),zk(14),zk(15),zk(16),zk(17),zk(18),zk(19));
Gk_Z = Gk_f(zk(14),zk(16),zk(18));
Jxw_Z = Jxw_f(zk(6));
dJxw_Z = dJxw_f(zk(6));
JrvA_Z = JrvA_f(zk(6),zk(12),zk(14),zk(16),zk(18));
dJrvA_Z = dJrvA_f(zk(6),zk(7),zk(12),zk(13),zk(14),zk(15),zk(16),zk(17),zk(18),zk(19));

R_Z = R_f(zk(6),zk(12),zk(14),zk(16),zk(18));
% torque
u_D = dJxw_Z*zk(9:2:end)+Jxw_Z*u;
tau = Mk_Z*u_D+Vk_Z+Gk_Z;
torque_implement = [torque_implement  double(tau) ];

% force
% Jit = pinv(Jdk_Z)';
% Ji = pinv(Jk_Z);
% Mx = Jit*Mk_Z*Ji;
% Vx = Jit*(Vk_Z-Mk_Z*Ji*Jdk_Z*zk(9:2:end));
% Gx = Jit*Gk_Z;
% thv = [Zk(5) Zk(7) Zk(13) Zk(15) Zk(17) Zk(19) ]';
% Jit = pinv(Jrv_Z)';
% Ji = pinv(Jrv_Z);
% Mx = Jit*Mk_Z*Ji;
% Vx = Jit*(Vk_Z-Mk_Z*Ji*dJrv_Z*thv);
% Gx = Jit*Gk_Z;
% 
% Xdd = dJrv_Z*thv+Jk_Z*u;
% force = Mx*Xdd+Vx+Gx;

Jit = pinv(JrvA_Z)';

Mx = Jit*Mk_Z;
Vx = Jit*Vk_Z;
Gx = Jit*Gk_Z;

force_l = inv(R_Z)*(Mx*u_D+Vx+Gx);
forcel_implement = [forcel_implement double(force_l)];

force_g = Mx*u_D+Vx+Gx;
forceg_implement = [forceg_implement double(force_g)];

angle_implement = [angle_implement zk];

traj_ = [zk(1,:);
         zk(2,:)];

theta_ = [zk(6,:);
          zk(12,:);
          zk(14,:);
          zk(16,:);
          zk(18,:)];

[end_effector]=get_end(theta_out,[x_out';y_out'],robot);



% next step
[zk_, A, B ] = LinKin(zk, u_ctrl, dt);
zk = zk_;

(JrvA_Z*Jxw_Z)'

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
%%
% robot visualize              
figure
gap = 2;
plot_MM5(ss,theta_implement(1:end-1,:),traj_implement,robot,tb3,gap,0)
%plot_implement(ss,theta_implement,traj_implement,robot,tb3,gap,0)
%%
% torque
figure



plot(torque_implement(3,:))
hold on
plot(torque_implement(4,:))
plot(torque_implement(5,:))
plot(torque_implement(6,:))
plot(torque_implement(7,:))
xlabel('Time steps')
ylabel('Torque')
% plot(torque_implement1(1,:))
% hold on
% plot(torque_implement1(2,:))
legend('\theta_1', '\theta_2','\theta_3', '\theta_4','\theta_5')

% degree
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
%%
% force
figure

plot(forceg_implement(1,:))
hold on
plot(forceg_implement(2,:))
plot(forceg_implement(3,:))

xlabel('Time step')
ylabel('Force_w')
legend('X_{e,w}', 'Y_{e,w}','Z_{e,w}')

figure

plot(forcel_implement(1,:))
hold on
plot(forcel_implement(2,:))
plot(forcel_implement(3,:))

xlabel('Time step')
ylabel('Force_l')
legend('X_{e,5}', 'Y_{e,5}','Z_{e,5}')