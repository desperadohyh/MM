function [ Mk, Vk, Gk, robot] = joint_torque_r(robot,obs_arm)

g = 9.81;
cap = robot.cap;
DH = robot.DH;
base = robot.base;
theta = robot.theta;
R_ = robot.R_;
M = robot.M;
nlink = robot.nlink;
T = robot.T; % distance between coordinates
Tc = robot.Tc;
l = robot.l;
lc = robot.lc;
m = robot.m;
Z0 = robot.Z0;
R_inv = cell(1,nlink);
Ic = robot.Ic;
obs = obs_arm;


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
theta = Z0(1:2:end);
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
        
        if i == 3
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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% xA, yA, xAd, yAd, v,   t1, t1d, tR, tRd, tL,  
% tLd, t2, t2d, t3, t3d,   t4, t4d,t5,t5d 



%Vo_l = zeros(3,nlink);
%Vo_g = zeros(3,nlink);
%Wo_l = zeros(3,nlink);
%Vc = zeros(3,nlink);
%Keng = zeros(nlink,1);

%% Kinetic Energy
Wo_l(:,1) = [0;0;Z0(2)]; %angular velocity of the origion w.r.t local coordinate 

for i = 1:nlink
    R_inv{i} = inv(R_{i});
end
robot.R_inv =  R_inv;
Vc(:,1) = sym([ 0  0  0]');  %%%%%%%%%%% need to be initialized with model change%%%%%%%%%%%%%%%
Vo_l(:,1) = R_inv{i}*Vc(:,1);
Keng(1) = 0.5*m(1)*Vc(:,1)'*Vc(:,1) + 0.5*Wo_l(:,1)'*Ic{1}*Wo_l(:,1);

for i = 1:nlink-1
    Vo_l(:,i+1) = R_inv{i} * (Vo_l(:,i) + cross(Wo_l(:,i),T(:,i+2)));
    Vo_g(:,i+1) = M{i+2}(1:3,1:3)*Vo_l(:,i+1);
    Wo_l(:,i+1) = R_inv{i+1}*Wo_l(:,i) + [ 0 0 Z0(i*2)]';
    Vc(:,i+1) = Vo_g(:,i+1) + M{i+2}(1:3,1:3)*(cross(Wo_l(:,i+1),Tc(:,i+1)));
    Keng(i+1) = 0.5*m(i+1)*Vc(:,i+1)'*Vc(:,i+1) + 0.5*Wo_l(:,i+1)'*Ic{i+1}*Wo_l(:,i+1);
end

KENG = sum(Keng);

%% Potential Energy


ueng(1) = m(1)*g*(M{1}(3,4) + lc(1)*sin(pi/2 - (Z0(1)+0.2450)));
ueng(2) = m(2)*g*(M{1}(3,4) + l(1)*sin(pi/2 - (Z0(1)+0.2450)) - lc(2)*sin(Z0(1) + Z0(3)));

ueng(1) = m(1)*g*lc(1);
ueng(2) = m(2)*g*(M{2}(3,4) + lc(2)*sin(pi/2 - (Z0(3)+0.2450)));
ueng(3) = m(3)*g*(M{2}(3,4) + l(2)*sin(pi/2 - (Z0(3)+0.2450)) - lc(3)*sin(Z0(3) + Z0(5)));
ueng(4) = m(4)*g*(M{2}(3,4) + l(2)*sin(pi/2 - (Z0(3)+0.2450)) - l(3)*sin(Z0(3) + Z0(5)) - lc(4)*sin(Z0(3) + Z0(5) + Z0(7)));

UENG = sum(ueng);
%% Inner-torque


thetadd = hessian(KENG,[Z0(2) Z0(4) Z0(6) Z0(8)]);
Mk = double(subs(thetadd,Z0,robot.z0_));

Vk_2 = gradient(KENG,[Z0(2) Z0(4) Z0(6) Z0(8) ]);
Vk_1 = jacobian(Vk_2,[Z0(1) Z0(3) Z0(5) Z0(7)]);
thetad = Vk_1*robot.z0_(2:2:end)+gradient(-KENG,[Z0(1) Z0(3) Z0(5) Z0(7)]);
Vk = double(subs(thetad,Z0,robot.z0_));

uthetad = gradient(UENG,[Z0(1) Z0(3) Z0(5) Z0(7)]);
Gk = double(subs(uthetad,Z0,robot.z0_));



end