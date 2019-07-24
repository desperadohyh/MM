function [A_tau, b_tau, robot] = get_joint_torque(robot,obs_arm)

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
end