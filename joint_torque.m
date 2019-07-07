clc
clear all
close all
fighandle = [];
fighandle(1) = figure(1); hold on;
set(gcf, 'position', [0 0 500 500]);
fighandle(2) = figure(2); hold on;

    
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
robot=robotproperty_MMD(4, z0_, Ts);
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
robot.Z0 = sym('z',[19 1]);
robot.z0_ = zeros(19,1);
robot.nTherta = 6;

%%
[d, linkid, A_tau, b_tau, robot] = joint_torque1(robot,obs_arm);




function [d, linkid, A_tau, b_tau, robot] = joint_torque1(robot,obs_arm)

g = 9.81;
cap = robot.cap;
DH = robot.DH;
base = robot.base;
theta = robot.theta;
R_ = robot.R_;
M = robot.M;
nlink = robot.nlink;
T = robot.T;
Tc = robot.Tc;
l = robot.l;
lc = robot.lc;
m = robot.m;
Z0 = robot.Z0;
R_inv = cell(1,nlink);
Ic = robot.Ic;
obs = obs_arm;

T = [0   0 -0.15    0   0.03 0.13;
     0   0   0      0  -0.12   0 ;
     0   0   0.0  0.04     0    0];

%Vo_l = zeros(3,nlink);
%Vo_g = zeros(3,nlink);
%Wo_l = zeros(3,nlink);
%Vc = zeros(3,nlink);
%Keng = zeros(nlink,1);

%% Kinetic Energy
Wo_l(:,1) = [0;0;Z0(7)];

for i = 1:nlink
    R_inv{i} = inv(R_{i});
end

Vc(:,1) = [ Z0(9)+lc(1)*Z0(7)*sin(Z0(6))  Z0(11)-lc(1)*Z0(7)*cos(Z0(6))  0]';
Vo_l(:,1) = R_inv{i}*Vc(:,1);
Keng(1) = 0.5*m(1)*Vc(:,1)'*Vc(:,1) + 0.5*Wo_l(:,1)'*Ic{1}*Wo_l(:,1);

for i = 1:nlink-1
    Vo_l(:,i+1) = R_inv{i} * (Vo_l(:,i) + cross(Wo_l(:,i),T(:,i+2)));
    Vo_g(:,i+1) = M{i+2}(1:3,1:3)*Vo_l(:,i+1);
    Wo_l(:,i+1) = R_inv{i+1}*Wo_l(:,i) + [ 0 0 Z0(11+i*2)]';
    Vc(:,i+1) = Vo_g(:,i+1) + M{i+2}(1:3,1:3)*(cross(Wo_l(:,i+1),Tc(:,i+1)));
    Keng(i+1) = 0.5*m(i+1)*Vc(:,i+1)'*Vc(:,i+1) + 0.5*Wo_l(:,i+1)'*Ic{i+1}*Wo_l(:,i+1);
end

KENG = sum(Keng);

%% Potential Energy

ueng(1) = sym(m(1)*g*0.1);
ueng(2) = m(2)*g*(0.1 + lc(2));
ueng(3) = m(3)*g*(M{3}(3,4) + lc(3)*sin(pi/2 - Z0(14)));
ueng(4) = m(4)*g*(M{3}(3,4) + l(3)*sin(pi/2 - Z0(14)) - lc(4)*sin(Z0(14) + Z0(16)));
ueng(5) = m(5)*g*(M{3}(3,4) + l(3)*sin(pi/2 - Z0(14)) - l(4)*sin(Z0(14) + Z0(16)) - lc(5)*sin(Z0(14) + Z0(16) + Z0(18)));

UENG = sum(ueng);
%% Inner-torque


thetadd = hessian(KENG,[Z0(9) Z0(11) Z0(13) Z0(15) Z0(17) Z0(19)]);
A_tau = double(subs(thetadd,Z0,robot.z0_));

thetad = gradient(-KENG,[Z0(8) Z0(10) Z0(12) Z0(14) Z0(16) Z0(18)]);
thetad = thetad + gradient(UENG,[Z0(8) Z0(10) Z0(12) Z0(14) Z0(16) Z0(18)]);
b_tau = double(subs(thetad,Z0,robot.z0_));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nstate = size(DH,1);
for i=1:nstate
DH(i,1)=theta(i);
end%theta,d,a,alpha
d = Inf;
if size(base,2)>1
    base=base';
end

%pos=CapPos(base,DH,cap);

% Ulternative for CapPos

RoCap = cap;
nlink=size(theta,1);
pos=cell(1,nlink);
M=cell(1,nlink+1); M{1}=eye(4);
R_=cell(1,nlink); R_{1}=eye(3);
for i=2:nlink+1
        % R in book
        R=[cos(theta(i-1)) -sin(theta(i-1))  0;
            sin(theta(i-1)) cos(theta(i-1))  0;
              0                        0                 1];
        % T in book
        %T=[DHn(i-1,3);-sin(DHn(i-1,4))*DHn(i,2);cos(DHn(i-1,4))*DHn(i,2)];
        
        if i == 4
            Rx=[1     0             0        ;  
                0  cos(-0.5*pi) -sin(-0.5*pi); 
                0  sin(-0.5*pi) cos(-0.5*pi) ];
                   
            R = Rx*R;
        end
        R_{i-1} = R;
        M{i}=M{i-1}*[R T(:,i); zeros(1,3) 1]; ; 
        for k=1:2
         pos{i-1}.p(:,k)=M{i}(1:3,1:3)*RoCap{i-1}.p(:,k)+M{i}(1:3,4)+base;
        end
end

robot.R_ = R_;
robot.M = M;

for i=1:nstate
    
    [dis, points] = distLinSeg(pos{i}.p(:,1),pos{i}.p(:,2), obs(:,1),obs(:,2));
    if norm(dis)<0.0001
        dis = -norm(points(1:3,1)-pos{i}.p(:,2));%%%%%%%%
    end        
    if dis < d
        d = dis;
        linkid=i;
    end
end
end