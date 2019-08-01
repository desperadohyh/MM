function [ Mk, Vk, Gk, robot] = get_joint_torque_new(robot)
% xA, yA, xAd, yAd, v,   t1, t1d, tR, tRd, tL,  
% tLd, t2, t2d, t3, t3d,   t4, t4d,t5,t5d 

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
r = robot.r;
d = robot.d;
Ts = robot.delta_t;
z0_ = robot.z0_;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

theta =[Z0(6) ; Z0(12:2:end)];
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

th1 = z0_(6)+Ts*(r/d)*(Z0(9)-Z0(11));
xda = (r/2)*(-(z0_(9)+z0_(11))*sin(z0_(6))*th1 + cos(z0_(6))*Z0(9) + cos(z0_(6))*Z0(11));
yda = (r/2)*(-(z0_(9)+z0_(11))*sin(z0_(6))*th1 + cos(z0_(6))*Z0(9) + cos(z0_(6))*Z0(11));

Vc(:,1) = [ xda+lc(1)*Z0(7)*sin(Z0(6))  yda-lc(1)*Z0(7)*cos(Z0(6))  0]';
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
ueng(3) = m(3)*g*(M{3}(3,4) + lc(3)*sin(pi/2 - (Z0(14)+0.2450)));
ueng(4) = m(4)*g*(M{3}(3,4) + l(3)*sin(pi/2 - (Z0(14)+0.2450)) - lc(4)*sin(Z0(14) + Z0(16)));
ueng(5) = m(5)*g*(M{3}(3,4) + l(3)*sin(pi/2 - (Z0(14)+0.2450)) - l(4)*sin(Z0(14) + Z0(16)) - lc(5)*sin(Z0(14) + Z0(16) + Z0(18)));

UENG = sum(ueng);
%% Inner-torque


thetadd = hessian(KENG,[Z0(9) Z0(11) Z0(13) Z0(15) Z0(17) Z0(19)]);
Mk  = double(subs(thetadd,Z0,z0_));

Vk_2 = gradient(KENG,[Z0(9) Z0(11) Z0(13) Z0(15) Z0(17) Z0(19)]);
Vk_1 = jacobian(Vk_2,[Z0(8) Z0(10) Z0(12) Z0(14) Z0(16) Z0(18)]);
thetad = Vk_1*z0_(9:2:end)+gradient(-KENG,[Z0(8) Z0(10) Z0(12) Z0(14) Z0(16) Z0(18)]);
Vk = double(subs(thetad,Z0,z0_));

uthetad = gradient(UENG,[Z0(8) Z0(10) Z0(12) Z0(14) Z0(16) Z0(18)]);
Gk = double(subs(uthetad,Z0,z0_));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end