clc
clear all
close all

%% Record movie
fighandle = [];
fighandle(1) = figure(1); hold on;
%set(gcf, 'position', [0 0 500 500]);
video = VideoWriter('peaks2_st.avi');
video.FrameRate = 5;
open(video);

%%

dt = 0.1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%
gen_ref_MMD_0726
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Arm parameters
robot=robotproperty_MMD(4, z0_, Ts);
nlink = 5;
R_z90 = [cos(-pi/2)  -sin(-pi/2)  0;
        sin(-pi/2)  cos(-pi/2)   0;
        0              0         1];
R_all = kron(eye(3),R_z90);


%%
%load('MVG_3.mat')
ss = 30;

torque_implement = [];
torque_implement1 = [];
angle_implement = [];
forcel_implement = [];
forceg_implement = [];

F_all_e = [];
tau_all_e = [];
u_all_e = [];

F_all_th = [];
tau_all_th = [];
u_all_th = [];

robot.base = [0 0 0]';


z0_k = [-0.1562 0  0 0 0    0 0 0 0 0     0 0 0 pi/4 0      -pi/8 0 -pi/8 0 ]';

% generate refrence

inputi = 3;

xlim = 5;
         
% initials
load('IK_open_pidref.mat');

% end effector reference
end_effectorr = Ref.traj_e;
X_all = Ref.traj_e;
%load('open_data.mat')
%cc = load('open_please');
%X_all = cc.cc;
pend = [0 0 0.06]';
dpend = [0  0  0]';
zk_d0 = z0_k;
zk = z0_k;
u_D = zeros(3,1);
u_ctrl = zeros(6,1);
% Model_d
Md = 0.4667;
Bd = 2.25;
Kd = 2;
Error_e = [];
Error_th = [];
end_all = [];

%X_all(3,:) = 0.06*ones(1,size(X_all,2));
X_all(3,:) = 0.1*ones(1,size(X_all,2));
v = 0.05;
num = round((xlim -X_all(1,end))/(v*dt));
gg = [linspace(X_all(1,end),xlim ,num );
     X_all(2,end)*ones(1,num);
     X_all(3,end)*ones(1,num);
     v*ones(1,num);
     0*ones(1,num);
     0*ones(1,num);
     0*ones(1,num);
     0*ones(1,num);     
     0*ones(1,num)];
      
X_all =[ [X_all; Ref.dee_e;Ref.ddee_e] gg];


% motor angle and base reference
Thv_d = [Ref.traj(1:2,:); Ref.theta_open]; 
Thvd_d = [Ref.deej(1:2,:); Ref.deet];
Thvdd_d = [Ref.ddeej(1:2,:); Ref.ddeet];

for steps = 1:ss

% desired reference    
% u_d = alpha_all(:,steps);
% [zk_d, A, B ] = LinKin(zk_d0, u, dt);

% desired reference
    % End-effector 
xdd_d = [X_all(7:9,steps)]; 
xd_d = [X_all(4:6,steps)];
x_d = [X_all(1:3,steps)];
    % angles
thv_d = [Thv_d(:,steps)];
thvd_d = [Thvd_d(:,steps)];
thvdd_d = [Thvdd_d(:,steps)];



Mk_Z = Mk_f(zk(6),zk(12),zk(14),zk(16),zk(18));
Vk_Z = Vk_f(zk(3),zk(4),zk(6),zk(7),zk(12),zk(13),zk(14),zk(15),zk(16),zk(17),zk(18),zk(19));
Gk_Z = Gk_f(zk(14),zk(16),zk(18));
Jxw_Z = Jxw_f(zk(6));
dJxw_Z = dJxw_f(zk(6),zk(7));
JrvA_Z = JrvA_f(zk(6),zk(12),zk(14),zk(16),zk(18));
dJrvA_Z = dJrvA_f(zk(6),zk(7),zk(12),zk(13),zk(14),zk(15),zk(16),zk(17),zk(18),zk(19));

R_Z = R_f(zk(6),zk(12),zk(14),zk(16),zk(18));
% torque
% u_D = dJxw_Z*zk(9:2:end)+Jxw_Z*u;
% tau = Mk_Z*u_D+Vk_Z+Gk_Z;
% torque_implement = [torque_implement  double(tau) ];

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



traj_ = [zk(1,:);
         zk(2,:)];

theta_ = [zk(6,:);
          zk(12,:);
          zk(14,:);
          zk(16,:);
          zk(18,:)];

[end_effector]=get_end(theta_,traj_,robot);

end_all = [ end_all  end_effector];

% current end-effector state
xdd = u_D;
xd = JrvA_Z*Jxw_Z*zk(9:2:end);
x = end_effector;

% current thate and base state

th = [zk(1) zk(2) zk(6) zk(12) zk(14) zk(16) zk(18)]';
thd = [zk(3) zk(4) zk(7) zk(13) zk(15) zk(17) zk(19)]';
thdd = dJxw_Z*zk(9:2:end) + Jxw_Z*u_ctrl;

% End-effector Error
error_e = [x_d - x; xd_d - xd; xdd_d - xdd];
Error_e = [Error_e  error_e];

error_th = [thv_d - th; thvd_d - thd; thvdd_d - thdd];
Error_th = [Error_th  error_th];




  
% control input
gall = [1 2 0];
gall = gall/norm(gall);

% End-efector tracking
gPID = [2 3 1.01];
gPID = gPID/norm(gPID);
%F_ctrl = (Vx+Mx*dJxw_Z*zk(9:2:end))+Gx+Mx*pinv(JrvA_Z)*(gPID(3)*xdd_d-(1/Md)*(gPID(2)*Bd*(xd - xd_d)+ gPID(1)*Kd*(x - x_d)));
F_ctrl = 0.001*[9.81/1.5 0 0]';%gPID(1)*(x_d - x) + gPID(2)*(xd_d - xd);
tau_ctrl = (JrvA_Z*Jxw_Z)'*F_ctrl;
u_ctrl_e = 50*diag([0  0  7  3  4  5])*tau_ctrl ;
%u_ctrl = pinv(Jxw_Z)*Mk_Z*Jxw_Z*(tau_ctrl - pinv(Jxw_Z)*(Vk_Z - Gk_Z - Mk_Z*dJxw_Z*zk(9:2:end)));
%u_ctrl = pinv(Jxw_Z)*(pinv(Mk_Z)*(tau_ctrl-Vk_Z-Gk_Z)-dJxw_Z*zk(9:2:end));
F_all_e = [F_all_e  F_ctrl ];
tau_all_e = [tau_all_e tau_ctrl];
u_all_e = [u_all_e u_ctrl_e];

% angle and base tracking

thPID = [ 1 3 0];
thPID = thPID/norm(thPID); 
rate = [40  40 40 7  3  4  5];
F_ctrl = diag(rate/norm(rate))*(thPID(1)*(thv_d - th) + thPID(2)*(thvd_d - thd));
tau_ctrl = (Jxw_Z)'*F_ctrl;
u_ctrl_th = diag([4000  4000  7  3  4  5])*tau_ctrl ;

F_all_th = [F_all_th  F_ctrl ];
tau_all_th = [tau_all_th tau_ctrl];
u_all_th = [u_all_th u_ctrl_th];



u_ctrl = gall(1)*u_ctrl_e + gall(2)*u_ctrl_th;



% next step
u_ = dJxw_Z*zk(9:2:end)+Jxw_Z*u_ctrl;
force_l = inv(R_Z)*(Mx*u_+Vx+Gx);
forcel_implement = [forcel_implement double(force_l)];

force_g = Mx*u_+Vx+Gx;
forceg_implement = [forceg_implement double(force_g)];
zk;
angle_implement = [angle_implement zk];
exf = [];
[zk_, A, B ] = LinKin_exf(zk, u_ctrl, dt,exf);

zk = zk_;





z0_k = zk;
%pause
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
figure(fighandle(1));
plot3(X_all(1,1:ss),X_all(2,1:ss),X_all(3,1:ss))
 xlabel('x[m]')
    ylabel('y[m]')
    zlabel('z[m]')
hold on
plot3(end_effectorr(1,1:ss),end_effectorr(2,1:ss),end_effectorr(3,1:ss),'-r*')
%%
gap = 1;
end_implement = plot_MM5(ss,theta_implement(1:end-1,:),traj_implement,robot,tb3,gap,0,video,fighandle);
hold on
plot3(X_all(1,1:ss),X_all(2,1:ss),X_all(3,1:ss));
plot3(end_implement(1,1:ss),end_implement(2,1:ss),end_implement(3,1:ss),'-b*')

frame = getframe(fighandle(1));
writeVideo(video,frame);
close(video);
%plot_implement(ss,theta_implement,traj_implement,robot,tb3,gap,0)
%%
% torque
% figure
% 
% 
% 
% plot(torque_implement(3,:))
% hold on
% plot(torque_implement(4,:))
% plot(torque_implement(5,:))
% plot(torque_implement(6,:))
% plot(torque_implement(7,:))
% xlabel('Time steps')
% ylabel('Torque')
% plot(torque_implement1(1,:))
% hold on
% plot(torque_implement1(2,:))
% legend('\theta_1', '\theta_2','\theta_3', '\theta_4','\theta_5')

% error
figure
tt = 1:ss;
plot(tt*dt,Error_e(1:6,:));

legend('e_{x}', 'e_{y}','e_{z}', 'e_{Vx}', 'e_{Vy}','e_{Vz}','e_{ax}', 'e_{ay}','e_{az}')

figure
tt = 1:ss;
plot(tt*dt,Error_th(1:7,:));

legend('e_x','e_y','e_{\theta_A}','e_{\theta_2}','e_{\theta_3}','e_{\theta_4}','e_{\theta_5}')

% degree
figure
angle_implement_deg = (180/pi).*angle_implement;

plot(angle_implement_deg(6,:))
hold on
plot(angle_implement_deg(12,:))
plot(angle_implement_deg(14,:))
plot(angle_implement_deg(16,:))
plot(angle_implement_deg(18,:))

legend('\theta_1', '\theta_2','\theta_3', '\theta_4','\theta_5')
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